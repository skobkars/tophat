/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "AirspaceWarningMonitor.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Asset.hpp"
#include "Audio/Sound.hpp"
#include "Dialogs/Airspace/AirspaceWarningDialog.hpp"
#include "Event/Idle.hpp"
#include "Operation/Operation.hpp"
#include "PageActions.hpp"
#include "Widget/QuestionWidget.hpp"
#include "Form/ActionListener.hpp"
#include "Language/Language.hpp"
#include "Engine/Airspace/AirspaceWarning.hpp"
#include "Engine/Airspace/AirspaceWarningManager.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Airspace/ProtectedAirspaceWarningManager.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "Blackboard/DeviceBlackboard.hpp"

class AirspaceWarningWidget final
  : public QuestionWidget, private ActionListener {
  enum Action {
    ACK,
    ACK_DAY,
    MORE,
  };

  AirspaceWarningMonitor &monitor;
  ProtectedAirspaceWarningManager &manager;

  const AbstractAirspace &airspace;
  AirspaceWarning::State state;

  StaticString<256> buffer;

  gcc_pure
  const TCHAR *MakeMessage(const AbstractAirspace &airspace,
                           AirspaceWarning::State state,
                           const AirspaceInterceptSolution &solution) {
    if (state == AirspaceWarning::WARNING_INSIDE)
      buffer.Format(_T("%s: %s"), _("Inside airspace"), airspace.GetName());
    else
      buffer.Format(_T("%s: %s (%s)"), _("Near airspace"), airspace.GetName(),
                    FormatTimespanSmart(int(solution.elapsed_time),
                                        2).c_str());

    return buffer;
  }

public:
  AirspaceWarningWidget(AirspaceWarningMonitor &_monitor,
                        ProtectedAirspaceWarningManager &_manager,
                        const AbstractAirspace &_airspace,
                        AirspaceWarning::State _state,
                        const AirspaceInterceptSolution &solution)
    :QuestionWidget(MakeMessage(_airspace, _state, solution), *this),
     monitor(_monitor), manager(_manager),
     airspace(_airspace), state(_state) {
    AddButton(_("ACK"), ACK);
    AddButton(_("ACK Day"), ACK_DAY);
    AddButton(_("More"), MORE);
  }

  ~AirspaceWarningWidget() {
    assert(monitor.widget == this);
    monitor.widget = nullptr;
  }

  bool Update(const AbstractAirspace &_airspace,
              AirspaceWarning::State _state,
              const AirspaceInterceptSolution &solution) {
    if (&_airspace != &airspace)
      return false;

    state = _state;
    SetMessage(MakeMessage(airspace, state, solution));
    return true;
  }

private:
  /* virtual methods from class ActionListener */
  void OnAction(int id) override;
};

void
AirspaceWarningWidget::OnAction(int id)
{
  switch ((Action)id) {
  case ACK:
    if (state == AirspaceWarning::WARNING_INSIDE)
      manager.AcknowledgeInside(airspace);
    else
      manager.AcknowledgeWarning(airspace);
    monitor.Schedule();
    PageActions::RestoreTop();
    break;

  case ACK_DAY:
    manager.AcknowledgeDay(airspace);
    monitor.Schedule();
    PageActions::RestoreTop();
    break;

  case MORE:
    dlgAirspaceWarningsShowModal(manager);
    return;
  }
}

void
AirspaceWarningMonitor::Reset()
{
  const auto &calculated = CommonInterface::Calculated();

  last = calculated.airspace_warnings.latest;
}

void
AirspaceWarningMonitor::HideWidget()
{
  if (widget != nullptr)
    PageActions::RestoreTop();
  assert(widget == nullptr);
}

void
AirspaceWarningMonitor::Check()
{
  const auto &calculated = CommonInterface::Calculated();

  if (widget == nullptr && calculated.airspace_warnings.latest == last)
    return;

  /* there's a new airspace warning */

  last = calculated.airspace_warnings.latest;
  auto *airspace_warnings = GetAirspaceWarnings();

  if (airspace_warnings == nullptr) {
    alarmClear();
    HideWidget();
    return;
  }

  replayAlarmSound(*airspace_warnings);

  if (!HasPointer()) {
    /* "classic" list-only view for devices without touch screen */

    if (dlgAirspaceWarningVisible())
      /* already visible */
      return;

    // un-blank the display, play a sound
    ResetUserIdle();
    alarmSet();

    // show airspace warnings dialog
    if (CommonInterface::GetUISettings().enable_airspace_warning_dialog)
      dlgAirspaceWarningsShowModal(*airspace_warnings, true);
    return;
  }

  const AbstractAirspace *airspace = nullptr;
  AirspaceWarning::State state;
  AirspaceInterceptSolution solution;

  {
    const ProtectedAirspaceWarningManager::Lease lease(*airspace_warnings);
    auto w = lease->begin();
    if (w != lease->end() && w->IsAckExpired()) {
      airspace = &w->GetAirspace();
      state = w->GetWarningState();
      solution = w->GetSolution();
    }
  }

  if (airspace == nullptr) {
    alarmClear();
    HideWidget();
    return;
  }

  if (CommonInterface::GetUISettings().enable_airspace_warning_dialog) {
    /* show airspace warning */
    if (widget != nullptr) {
      if (widget->Update(*airspace, state, solution))
        return;

      HideWidget();
    }

    widget = new AirspaceWarningWidget(*this, *airspace_warnings,
                                       *airspace, state, solution);
    PageActions::SetCustomTop(widget);
    alarmSet();
  }

  // un-blank the display, play a sound
  ResetUserIdle();
}

void
AirspaceWarningMonitor::alarmSet() {
  if (!alarm_active) {
    last_alarm_time = CommonInterface::Calculated().date_time_local;
    PlayResource(_T("IDR_WAV_AIRSPACE"));
    NullOperationEnvironment env;
    device_blackboard->PlayAlarm(env);
  }
  alarm_active = true;
}

void
AirspaceWarningMonitor::replayAlarmSound(ProtectedAirspaceWarningManager &airspace_warnings) {
  if (isAlarmActive()) {
    if (!airspace_warnings.IsEmpty()) {
      const auto current_time = CommonInterface::Calculated().date_time_local;
      const int seconds_since_last_alarm = current_time - last_alarm_time;

      // Process repetitive sound warnings if they are enabled in config
      const AirspaceWarningConfig &warning_config =
          CommonInterface::GetComputerSettings().airspace.warnings;
      if (warning_config.repetitive_sound) {
        unsigned tt_closest_airspace = timeToClosestAirspace(airspace_warnings);

        const unsigned sound_interval =((tt_closest_airspace * 3 / warning_config.warning_time) + 1) * 2;
        if (seconds_since_last_alarm > 0  && ((unsigned)seconds_since_last_alarm) >= sound_interval) {
          /* repetitive alarm sound */
          PlayResource(_T("IDR_WAV_AIRSPACE"));
          NullOperationEnvironment env;
          device_blackboard->PlayAlarm(env);
          last_alarm_time = current_time;
        }
      }
    }
  }
}

unsigned
AirspaceWarningMonitor::timeToClosestAirspace(ProtectedAirspaceWarningManager &airspace_warnings) {
  unsigned tt_closest_airspace = 3600;
  const ProtectedAirspaceWarningManager::Lease lease(airspace_warnings);

  for (auto it = lease->begin(), end = lease->end(); it != end; ++it) {
    /* Find smallest time to nearest and active aispace (cannot always rely
       on fact that closest airspace should be in the beginning of
       the list) */
    if (it->IsAckExpired()) {
      if ( it->GetWarningState() < AirspaceWarning::WARNING_INSIDE)
        tt_closest_airspace = std::min(tt_closest_airspace,
                                       unsigned(it->GetSolution().elapsed_time));
      else
        return 0;
      }
  }
  return tt_closest_airspace;
}

