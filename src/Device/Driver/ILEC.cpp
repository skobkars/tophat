/* Copyright_License {

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

#include "Device/Driver/ILEC.hpp"
#include "Device/Parser.hpp"
#include "Device/Driver.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/System.hpp"

//#include "Engine/Waypoint/Waypoints.hpp"
//#include "Engine/Task/Ordered/Points/OrderedTaskPoint.hpp"
#include "Components.hpp" // global way-points DB
#include "Interface.hpp" // ComputerSettings

// ...Pasted from SeeYou task processing
#include "Engine/Waypoint/Waypoints.hpp"
#include "Waypoint/WaypointReaderSeeYou.hpp"
#include "Task/ObservationZones/LineSectorZone.hpp"
#include "Task/ObservationZones/AnnularSectorZone.hpp"
#include "Task/ObservationZones/KeyholeZone.hpp"
#include "Engine/Task/Ordered/OrderedTask.hpp"
#include "Engine/Task/Ordered/Points/StartPoint.hpp"
#include "Engine/Task/Ordered/Points/FinishPoint.hpp"
#include "Engine/Task/Ordered/Points/AATPoint.hpp"
#include "Engine/Task/Ordered/Points/ASTPoint.hpp"
#include "Engine/Task/Factory/AbstractTaskFactory.hpp"
#include "Operation/Operation.hpp"
#include "Units/System.hpp"

#include "Protection.hpp"

#include "Task/ProtectedTaskManager.hpp"

#include "Task/ObservationZones/SymmetricSectorZone.hpp" // not used by CU ??
#include "LogFile.hpp" // debugging output only


#include <stdlib.h>
#include <math.h>

class ILECDevice : public AbstractDevice {
public:
  virtual bool ParseNMEA(const char *line, struct NMEAInfo &info) override;
};

static bool ParsePT(NMEAInputLine &line);
static bool ParseTSK(NMEAInputLine &line);

static bool
ReadSpeedVector(NMEAInputLine &line, SpeedVector &value_r)
{
  fixed norm, bearing;

  bool bearing_valid = line.ReadChecked(bearing);
  bool norm_valid = line.ReadChecked(norm);

  if (bearing_valid && norm_valid) {
    value_r.norm = Units::ToSysUnit(norm, Unit::KILOMETER_PER_HOUR);
    value_r.bearing = Angle::Degrees(bearing);
    return true;
  } else
    return false;
}

/**
 * Parse a "$PILC,PDA1" sentence.
 *
 * Example: "$PILC,PDA1,1489,-3.21,274,15,58*7D"
 *
 */
static bool
ParsePDA1(NMEAInputLine &line, NMEAInfo &info)
{
  // altitude [m], already corrected for altimeter setting (not raw pressure-altitude)
  int altitude;
  if (line.ReadChecked(altitude))
    info.ProvideBaroAltitudeTrue(fixed(altitude));
  // total energy vario [m/s]
  fixed vario;
  if (line.ReadChecked(vario))
    info.ProvideTotalEnergyVario(vario);
  // wind direction [degrees, kph]
  // wind measurement confidence [0..100]
  SpeedVector SN10_wind_measurement;
  int wind_confidence;
  bool wind_measurement_provided  = ReadSpeedVector(line, SN10_wind_measurement);
       wind_measurement_provided &= line.ReadChecked(wind_confidence);
  if (wind_measurement_provided) {
    // ToDo DRN: BugFix: Jamming SN10 wind reading into XCSoar wind regardless of confidence
    // and without asking pilot is a very, very bad idea... Better might be:
    // - consider SN10 setting and difference to current XCSoar setting, and pop question?
    // - copy SN10 setting?
    // - ignore estimates with confidence less than 65%?
    info.ProvideExternalWind(SN10_wind_measurement);
  }
  return true;
}

/**
 * Parse an SN10 "$PILC,POLAR" sentence
 *
 * The POLAR sentence gives the glider polar in TRUE speeds.
 * It is output periodically and if the pilot changes bugs/water
 * or the density-altitude changes significantly.
 * This gives the actual polar coefficients
 * (adjusted for bugs, water/weight, air density, true speed) as follows:
 * - Coefficients of the polar equations: sink(V) = a*V^2 + b*V + c
 * - Sink and V in meters/second TRUE speed.
 * - Sink will be a NEGATIVE number...
 * - 'a' is negative, 'b' positive, 'c' negative.
 *
 * v2.41 adds sqrt(sigma). Definitions reminder:
 * - Air Density Ratio (sigma) ::= current density / standard density
 * - TAS = CAS / sqrt(sigma)
 *
 * Example (at 3000 meters, hence sqrt(sigma) ~.83:
 *   $PILC,POLAR,-0.0012730,0.078267,-2.02283,0.8320*71
 */
static bool
ParsePOLAR(NMEAInputLine &line, NMEAInfo &info)
{
  double a,b,c;
  double sqrt_sigma = 1.0; // default is standard atmosphere at sea level
  bool polarProvided =
      line.ReadChecked(a) &
      line.ReadChecked(b) &
      line.ReadChecked(c) ;
  bool gcc_unused sqrt_sigma_provided = // not present prior v2.41
      line.ReadChecked(sqrt_sigma);
  if (polarProvided) {
    // ToDo DRN: jam ILEC SN10 polar into XCSoar - remember, TRUE speeds here!
    // ComputerSettings &settings_computer = CommonInterface::SetComputerSettings();
    // Set settings_computer.polar. ...
  }
  return false;
}

/**
 * Parse an SN10 "$PILC,SET" sentence
 *
 * The SET sentence (only when an included value changes) gives:
 * - altimeter setting in millibars (nowadays called hectoPascals)
 *
 * Example:
 *   $PILC,SET,1013.8*41
 */
static bool
ParseSET(NMEAInputLine &line, NMEAInfo &info) {
  double altimeter_setting_millibars;
  if (line.ReadChecked(altimeter_setting_millibars)) {
    LogFormat("QNH set to %f",altimeter_setting_millibars);
    info.settings.ProvideQNH(
        AtmosphericPressure::HectoPascal(altimeter_setting_millibars),
        info.clock);
    return true;
  }
  return false;
}

bool
ILECDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
//  RLD DEBUG
//  if (!VerifyNMEAChecksum(_line))
//    return false;


  NMEAInputLine line(_line);
  char type[16];
  line.Read(type, sizeof(type));
  if (StringIsEqual(type, "$PILC")) {
    line.Read(type, sizeof(type));
    if (StringIsEqual(type, "PDA1" )) return ParsePDA1 (line, info);
    if (StringIsEqual(type, "POLAR")) return ParsePOLAR(line, info);
    if (StringIsEqual(type, "SET"  )) return ParseSET  (line, info);
    if (StringIsEqual(type, "PT"   )) return ParsePT   (line);
    if (StringIsEqual(type, "TSK"  )) return ParseTSK  (line);
  }
  return false;
}

static Device *
ILECCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new ILECDevice();
}

const struct DeviceRegister ilec_driver = {
  _T("ILEC SN10"),
  _T("ILEC SN10"),
  0,
  ILECCreateOnPort,
};

// ===========================================================================

// Seriously hard-wired indices/counts for SN10 point array - Do not change!
enum {
  SN10_PTIDX_START    =  0,
  SN10_PTIDX_FINISH   = 11,
  SN10_PTIDX_FIRST_TP =  1,
  SN10_PTIDX_LAST_TP  = 10,
  SN10_PT_CNT         = 12,
};

/**
 * SN10 task is received piecemeal via multiple NMEA sentences, possibly spread
 * over a dozen seconds. This static structure accumulates the task info.
 * Notes:
 * - default ctors create valid empty point/task (and are used below)
 * - all heights and sizes in meters
 */
static struct SN10taskInfo_T { // Note: heights and sizes in meters
  // RLD How are these variables initialized and reset ?
  bool task_settings_received; // from SN10 $PILC,TSK sentence...
  bool xcsoar_task_declared; // a task has been declared into XCSoar from this info
  /* were any of the SN10 waypoints not found in the XCSoar wp database? */
  bool waypoint_database_requires_append;

  /// Task settings from SN10
  struct SN10taskSettings_T {
    enum TaskType_T {
      TASK_AREA,
      TASK_CLASSIC,
    } task_type;

    enum PointType_T {
      PT_NONE,
      PT_CYLINDER,
      PT_FAI,
      PT_LINE, /*start-finish only*/
    };

    int task_time_minutes; // PST time (area, USA MAT, OLC club, etc)
    PointType_T start_type;
    int start_radius; // line width, cylinder radius, in Meters
    int start_height; // Not implemented in SN10 v2.40; 0 indicates Do-Not-Use

    PointType_T finish_type;
    int finish_radius; // line width, cylinder radius, in Meters
    int finish_height_MSL; // set in SN10 via 'Fin'
    PointType_T turn_type; // FAI or cylinder, only used for racing tasks
    int turnpoint_radius; // relevant only for racing turns
    // Why BS said it was a mistake not to include default operator== in C++
    bool operator == (const SN10taskSettings_T &b) const {
      return task_type        == b.task_type           &&
          task_time_minutes   == b.task_time_minutes   &&
          start_type          == b.start_type          &&
          start_radius        == b.start_radius        &&
          start_height        == b.start_height        &&
          finish_type         == b.finish_type         &&
          finish_radius       == b.finish_radius       &&
          finish_height_MSL   == b.finish_height_MSL   &&
          turn_type           == b.turn_type           &&
          turnpoint_radius    == b.turnpoint_radius     ;
    }
    /// Translate NMEA input character to SN10 point type
    static bool GetPointType(PointType_T &result, char f);
    // Did SN10 send a start height?
    //tart height is not yet provided as of SN10 2.41
    bool StartHeightIsProvided() const {
      return start_height > 0;
    };
  } task_settings;

  /// Task points from SN10
  struct SN10PointRecd_T {
    bool point_received; // from SN10 $PILC,PT sentence...
    /// start, finish, and turn points (both area and classic)
    struct SN10pointDetail_T {
      /// Is this an actual point (as opposed to empty slot on SN10 task page)?
      bool is_non_nil_point;
      GeoPoint center;
      /// point name, 7 characters plus 0-terminator
      char name[8];
      bool is_achieved;
      /// UTC seconds past midnight (as reported by GPS), v2.40 provides this only for start point/start time
      int achieved_time;
      /// area target or actual location turned/started/finished
      GeoPoint actual_point;
      // Following for AAT points only...
      /// non-cylinder, with details below
      bool is_complex_AAT_shape;
      int inner_radius, outer_radius;
      /// start-finish angles, in true degrees; both 0 for cylinder
      int start_radial_degrees, end_radial_degrees;
      bool operator==(const SN10pointDetail_T &b) const {
          return is_non_nil_point  == b.is_non_nil_point     &&
              center               == b.center               &&
              strcmp(name,            b.name)==0             &&
              is_achieved          == b.is_achieved          &&
              achieved_time        == b.achieved_time        &&
              actual_point         == b.actual_point         &&
              is_complex_AAT_shape == b.is_complex_AAT_shape &&
              (!is_complex_AAT_shape ||
                (inner_radius == b.inner_radius &&
                 outer_radius == b.outer_radius &&
                 start_radial_degrees == b.start_radial_degrees &&
                 end_radial_degrees   == b.end_radial_degrees    )
              );
        };
    } ptbase;
  } pts[SN10_PT_CNT];

  // The minimal SN10 task is settings plus finish point[11],
  // but points (possibly nil) should be received for all 12 slots.
  /// Has a complete task been received from SN10?
  bool CompleteSN10TaskRecd() const {
    if (!task_settings_received)
      return false;
    for (int SN10_pt_idx = 0; SN10_pt_idx < SN10_PT_CNT; SN10_pt_idx++)
      if (!pts[SN10_pt_idx].point_received)
        return false;
    return  true;
  };

  /// Is this a club-mode task (no start or turn points, only finish)?
  bool IsClubmodeTask() const {
    return CompleteSN10TaskRecd() && !pts[SN10_PTIDX_START].ptbase.is_non_nil_point;
  }

  /**
   * @param SN10_pt_idx. index to SN10 task points
   * @return pointer to a newly created waypoint structure. or nullptr if error
   * caller is responsible for memory management
   */
  Waypoint* GetXCSoarWaypoint(int SN10_pt_idx);

  /**
   * @param wp
   * @param SN10_pt_idx.  Type of SN10 task point type
   * @return a newly allocated oz object
   * caller is responsible for memory management
   */
  ObservationZonePoint *GetNewObservationZone(Waypoint pt, int SN10_pt_idx);

  void UpdateXCSoarTaskFromSN10();

} SN10_task;

typedef SN10taskInfo_T::SN10PointRecd_T::SN10pointDetail_T PTbase_T;

/// Translate NMEA input character to SN10 point type
bool
SN10taskInfo_T::SN10taskSettings_T::GetPointType(PointType_T &result, char f)
{
  switch(f) {
    case 'N': result = SN10taskInfo_T::SN10taskSettings_T::PointType_T::PT_NONE; break; // SN10: no automatic/logger function
    case 'C': result = SN10taskInfo_T::SN10taskSettings_T::PointType_T::PT_CYLINDER; break;
    case 'F': result = SN10taskInfo_T::SN10taskSettings_T::PointType_T::PT_FAI; break;
    case 'L': result = SN10taskInfo_T::SN10taskSettings_T::PointType_T::PT_LINE; break;
    default:  return false;
  }
  return true;
}

// RLD Behavior question: should it use the Tophat wp name or the SN10 wp name
// when it finds a match in the wp database?
// Need to research behavior of the Tophat "Dedupe waypoints" process
// and consider when/if to call that.
Waypoint*
SN10taskInfo_T::GetXCSoarWaypoint(int SN10_pt_idx)
{
  assert(pts[SN10_pt_idx].point_received);
  if (!pts[SN10_pt_idx].point_received)
    return nullptr;

  SN10PointRecd_T::SN10pointDetail_T &pt = pts[SN10_pt_idx].ptbase;
  assert(pt.is_non_nil_point);
  assert(pt.center.IsValid());
  if (!pt.is_non_nil_point)
    return nullptr;
  if (!pt.center.IsValid())
    return nullptr;

  Waypoint* wp = nullptr;

  // See if the waypoint is already in the global waypoint DB
  const Waypoint* wp_from_db = way_points.GetNearest(pt.center, fixed(70));
  // If closest waypoint found and closer than 70m to the original
  if (wp_from_db != nullptr &&
    wp_from_db->location.Distance(pt.center) <= fixed(70))
  {
    wp = new Waypoint(wp_from_db->location);
    wp->name = wp_from_db->name;
  } else {
    wp = new Waypoint(pt.center);
    wp->name = pt.name;
    waypoint_database_requires_append = true;
    LogDebug(_T("SN10taskInfo_T::GetXCSoarWaypoint() created new wp: %s"), pt.name);
  }
  return wp;
}

ObservationZonePoint*
SN10taskInfo_T::GetNewObservationZone(Waypoint wp, int SN10_pt_idx)
{
  // modeled after SeeYou CUP task construction (src/Task/TaskFileSeeYou.cpp, see CreateOZ)
  assert(pts[SN10_pt_idx].point_received);
  SN10PointRecd_T::SN10pointDetail_T &pt = pts[SN10_pt_idx].ptbase;
  assert(pt.is_non_nil_point);

  // radius and OZ type depend on SN10 task type, and whether this is start or finish point
  int radius;
  SN10taskSettings_T::PointType_T SN10_point_type;
  bool is_turnpoint;
  switch(SN10_pt_idx) {
  case SN10_PTIDX_START:
    is_turnpoint = false;
    radius = task_settings.start_radius;
    SN10_point_type = task_settings.start_type;
    break;
  case SN10_PTIDX_FINISH:
    is_turnpoint = false;
    radius = task_settings.finish_radius;
    SN10_point_type = task_settings.finish_type;
    break;
  default:
    is_turnpoint = true;

    if (task_settings.task_type == SN10taskSettings_T::TASK_AREA) {
      radius = pt.outer_radius;
      if (radius == 0) {
        // during transition from racing to area task, updated points may not yet be received, hence 0 radius
        radius = 500; // prevent disasters with dummy 500 meter radius areas
      }
      SN10_point_type = SN10taskSettings_T::PT_CYLINDER;
    } else {
      radius = task_settings.turnpoint_radius;
      SN10_point_type = task_settings.turn_type;
    }
    break;
  }

  // Annular zone only happens for complex AAT turnpoint
  if (task_settings.task_type == SN10taskSettings_T::TASK_AREA && pt.is_complex_AAT_shape) {
    // WARNING: A donut (a non-zero inner radius with a FullCircle) causes assertion failure during XCSoar drawing...
    return new AnnularSectorZone(wp.location,
                                 pt.outer_radius,
                                 Angle::Degrees(pt.start_radial_degrees),
                                 Angle::Degrees(pt.end_radial_degrees),
                                 pt.inner_radius);
  };
  switch(SN10_point_type) {
  case SN10taskSettings_T::PT_CYLINDER:
    return new CylinderZone(wp.location, radius/*radius, in meters*/);
  case SN10taskSettings_T::PT_FAI:
    return SymmetricSectorZone::CreateFAISectorZone(wp.location, is_turnpoint);
  case SN10taskSettings_T::PT_LINE:

    return new LineSectorZone(wp.location, /*length =*/ fixed(2 * radius));
  case SN10taskSettings_T::PT_NONE:
  default:
    // SN10 can be set to 'None', which means disable automatic functions (used with some loggers, rarely).
    // In this case, just use fake cylinder - 'None' is unhelpful...
    return new CylinderZone(wp.location, 500/*radius, in meters*/);
  }
}

void
SN10taskInfo_T::UpdateXCSoarTaskFromSN10()
{
  LogFormat("SN10taskInfo_T::UpdateXCSoarTaskFromSN10() taskRecd:%u Club:%u",
            CompleteSN10TaskRecd(), IsClubmodeTask());
  if (!CompleteSN10TaskRecd())
    return;

  // An SN10 club mode task has only the target point; XCSoar's "OrderedTask" doesn't support this.
  if (IsClubmodeTask()) {
    // Create a "Goto" type task.
    std::unique_ptr<Waypoint> wp(GetXCSoarWaypoint(SN10_PTIDX_FINISH));
    if (wp == nullptr)
      return;
    protected_task_manager->DoGoto(*wp);
    xcsoar_task_declared = true;
    return;
  }

  // If not provided by SN10, abort.
  if (!pts[SN10_PTIDX_START].ptbase.is_non_nil_point) {
    LogFormat("SN10taskInfo_T::UpdateXCSoarTaskFromSN10() ABORT NO START");
    return;
  }

  // Proceed with a 'normal' SN10 task, which has a start, optional points, and a finish.
  // Task behavior from XCSoar user setup (things like autoMC behavior etc)
  ComputerSettings &settings_computer = CommonInterface::SetComputerSettings();
  TaskBehaviour &task_behaviour = settings_computer.task;
  OrderedTask *new_task = new OrderedTask(task_behaviour);
  TaskFactoryType xcsoar_task_type = TaskFactoryType::RACING;
  if (task_settings.task_type == SN10taskSettings_T::TaskType_T::TASK_AREA)
    xcsoar_task_type = TaskFactoryType::AAT;

  new_task->SetFactory(xcsoar_task_type);
  AbstractTaskFactory& fact = new_task->GetFactory();
  const TaskFactoryType fact_type = new_task->GetFactoryType();

  // Update task settings from SN10
  OrderedTaskSettings beh = new_task->GetOrderedTaskSettings();
  if (fact_type == TaskFactoryType::AAT) {
    // XCsoar doesn't use time limit for racing tasks, oh well...
    beh.aat_min_time = task_settings.task_time_minutes * 60; // seconds
  }

  // start height is not yet provided as of SN10 2.41
  if (task_settings.StartHeightIsProvided()) {
    beh.start_constraints.max_height = task_settings.start_height;
    beh.start_constraints.max_height_ref = AltitudeReference::MSL;
  }
  beh.finish_constraints.min_height = task_settings.finish_height_MSL;
  beh.finish_constraints.min_height_ref = AltitudeReference::MSL;

  new_task->SetOrderedTaskSettings(beh);
  bool new_task_constructed_ok = true;

  // Add start
  {
    assert(pts[SN10_PTIDX_START].ptbase.is_non_nil_point);
    std::unique_ptr<Waypoint> wp(GetXCSoarWaypoint(SN10_PTIDX_START));
    ObservationZonePoint* oz = GetNewObservationZone(*wp, SN10_PTIDX_START);
    OrderedTaskPoint *start_pt = fact.CreateStart(oz, *wp);
    new_task_constructed_ok &= fact.Append(*start_pt, false);
    delete start_pt;
  }

  // Add any turnpoints to new task
  for (int SN10_pt_idx = SN10_PTIDX_FIRST_TP;
      SN10_pt_idx <= SN10_PTIDX_LAST_TP; SN10_pt_idx++) {
    SN10PointRecd_T::SN10pointDetail_T &tp = pts[SN10_pt_idx].ptbase;
    if (!tp.is_non_nil_point)
      continue;
    std::unique_ptr<Waypoint> wp(GetXCSoarWaypoint(SN10_pt_idx));
    ObservationZonePoint *oz = GetNewObservationZone(*wp, SN10_pt_idx);
    OrderedTaskPoint *otp = nullptr;
    if (task_settings.task_type == SN10taskSettings_T::TASK_AREA)
      otp = fact.CreateAATPoint(oz, *wp);
    else
      otp = fact.CreateASTPoint(oz, *wp);

    new_task_constructed_ok &= fact.Append(*otp, false);
    delete otp;
  }

  // add Finish point to new task - always present
  {
    std::unique_ptr<Waypoint> wp(GetXCSoarWaypoint(SN10_PTIDX_FINISH));
    ObservationZonePoint* oz = GetNewObservationZone(*wp, SN10_PTIDX_FINISH);
    OrderedTaskPoint *finish_pt = fact.CreateFinish(oz, *wp);
    new_task_constructed_ok &= fact.Append(*finish_pt, false);
    delete finish_pt;
  }
  // Mark active point (start point if nothing yet achieved)
  int active_xcsoar_pt_idx = 0;
  for (int SN10_pt_idx=0; SN10_pt_idx<SN10_PT_CNT; SN10_pt_idx++)
    if (pts[SN10_pt_idx].ptbase.is_achieved)
      active_xcsoar_pt_idx++;
  // Overwrites complex AAT shape if nationality US! So don't do this: fact.MutateTPsToTaskType();
  new_task->ScanStartFinish(); // set task internal pointers to optional start and finish points
  assert(new_task_constructed_ok);
  assert(new_task->CheckTask()); // Check if the newly constructed task is valid

  if (waypoint_database_requires_append) {
    // this must be done in thread lock because it potentially changes the
    // waypoints database
    ScopeSuspendAllThreads suspend;
    new_task->CheckDuplicateWaypoints(way_points);
    way_points.Optimise();
    waypoint_database_requires_append = false;
    LogDebug("SN10taskInfo_T::UpdateXCSoarTaskFromSN10() appended/optimized waypoints database");
  }

  {
    ProtectedTaskManager::ExclusiveLease lease(*protected_task_manager);
    lease->Commit(*new_task);
    LogDebug("SN10taskInfo_T::UpdateXCSoarTaskFromSN10() COMMITTED! optimizable:%u AAT:%u",
             lease->GetOrderedTask().IsOptimizable(),
             task_settings.task_type == SN10taskSettings_T::TASK_AREA);
    if (lease->GetOrderedTask().IsOptimizable() &&
        task_settings.task_type == SN10taskSettings_T::TASK_AREA)
    {
      int xcsoar_pt_idx = 1; // 1 is first turnpoint (0 is the start point)
      for (int SN10_pt_idx = 1; SN10_pt_idx < SN10_PTIDX_FINISH; SN10_pt_idx++) {
        SN10PointRecd_T::SN10pointDetail_T &pt = pts[SN10_pt_idx].ptbase;
        if (!pt.is_non_nil_point)
          continue;
        if (pt.actual_point != GeoPoint::Zero()) {
          lease->SetTarget(xcsoar_pt_idx, pt.actual_point, true);
          lease->TargetLock(xcsoar_pt_idx, true);
        }
        xcsoar_pt_idx++;
      }
    } // aat actual point update
    lease->SetActiveTaskPoint(active_xcsoar_pt_idx);
    // ToDo DRN: update remaining task state info: start time
    //const AircraftState default_state;
    //lease->OverrideStartTime(default_state, 0);
  }
}

/**
 * Parse NMEA SN10 task settings sentence $PILC,TSK
 * @return false if error parsing sentence
 */
static bool
ParseTSK(NMEAInputLine &line)
{
  LogFormat("ParseTSK ENTER");
  SN10taskInfo_T::SN10taskSettings_T task_settings =
      SN10taskInfo_T::SN10taskSettings_T();
  // Example TSK: $PILC,TSK,A,300,L,8045,,N,3218,256,N,1448,*5D
  char task_type_code = line.ReadOneChar();
    switch(task_type_code) {
    case 'A':
      task_settings.task_type =
          SN10taskInfo_T::SN10taskSettings_T::TaskType_T::TASK_AREA; break;
    case 'R':
      task_settings.task_type =
          SN10taskInfo_T::SN10taskSettings_T::TaskType_T::TASK_CLASSIC; break;
    default:
      return false;
  }
  if (!line.ReadChecked(task_settings.task_time_minutes))
    return false;
  char start_type_code = line.ReadOneChar(); // "NCFL"; // None, Cylinder, FAI, Line
  if (!SN10taskInfo_T::SN10taskSettings_T::GetPointType(
      task_settings.start_type, start_type_code))
    return false;
  if (!line.ReadChecked(task_settings.start_radius))
    return false;

  // RLD Do we want to set Tophat start height to zero if none is sent from the SN10?
  line.ReadChecked(task_settings.start_height);
  LogDebug("Read Start_height:%i", task_settings.start_height);

  char finish_type_code = line.ReadOneChar(); // "NCFL"; // None, Cylinder, FAI, Line
  if (!SN10taskInfo_T::SN10taskSettings_T::GetPointType(
      task_settings.finish_type, finish_type_code))
    return false;
  if (!line.ReadChecked(task_settings.finish_radius))
    return false;
  if (!line.ReadChecked(task_settings.finish_height_MSL))
    return false;
  char turn_type_code = line.ReadOneChar(); // "NCF"; // None, Cylinder, FAI
  if (!SN10taskInfo_T::SN10taskSettings_T::GetPointType(
      task_settings.turn_type, turn_type_code))
    return false;
  if (!line.ReadChecked(task_settings.turnpoint_radius))
    return false;
  // If task settings are new, build and set new XCSoar task...
  if (SN10_task.xcsoar_task_declared && task_settings == SN10_task.task_settings)
    return true; // duplicate, nothing more to do
  SN10_task.task_settings_received = true;
  SN10_task.task_settings = task_settings;
  SN10_task.UpdateXCSoarTaskFromSN10();
  return true;
}

/// Parse NMEA SN10 point detail sentence $PILC,PT
// Todo: RLD: how do we remove a waypoint from the task; if name empty?
static bool ParsePT(NMEAInputLine &line) {
  PTbase_T pt = PTbase_T();
  // Example PTs:
  // - nil           $PILC,PT,5*27
  // - racing:       $PILC,PT,1,Fitchbr,4233.250,N,07145.533,W,N,*2B
  // - simple area:  $PILC,PT,3,Orange ,4234.200,N,07217.317,W,N,,4234.200,N,07217.317,W,7500*7D
  // - complex area: $PILC,PT,2,Lebanon,4337.567,N,07218.250,W,N,,4334.867,N,07218.250,W,35000,5000,135,225*38
  int pt_number; // ordinal, not index...
  if (!line.ReadChecked(pt_number)) {
    LogFormat("ParsePT ENTER / exit bad number");
    return false;
  }
  LogFormat("ParsePT ENTER #%i", pt_number);

  if (pt_number < 1 || pt_number > SN10_PT_CNT)
    return false;
  if (!line.IsEmpty()) {
    pt.is_non_nil_point = true;
    line.Read(pt.name, sizeof(pt.name));
    if (!NMEAParser::ReadGeoPoint(line, pt.center))
      return false;
    char achievedFlag = line.ReadOneChar();
    if (achievedFlag != 'Y' && achievedFlag != 'N')
      return false;
    pt.is_achieved = (achievedFlag == 'Y');
    line.ReadChecked(pt.achieved_time); // optional time this point was achieved (currently, only provided for start point/time)
    BrokenTime gcc_unused bAt = BrokenTime::FromSecondOfDay(pt.achieved_time);
    NMEAParser::ReadGeoPoint(line, pt.actual_point); // optional actual point (point actually turned, or planned point within turn area)
    if (line.ReadChecked(pt.outer_radius)) { // provided for all AAT points, but not for racing task turnpoints
      pt.is_complex_AAT_shape = // optional fields below only for complex areas
      line.ReadChecked(pt.inner_radius)
          && line.ReadChecked(pt.start_radial_degrees)
          && line.ReadChecked(pt.end_radial_degrees);
    };
  };
  // Have we already received this point info?
  SN10_task.pts[pt_number - 1].point_received = true; // prior check below for case of received nil point
  if (pt == SN10_task.pts[pt_number - 1].ptbase)
    return true; // parse succeeded, but nothing more to do (already received and processed this point)
  // This is new/changed, so reset task
  SN10_task.pts[pt_number - 1].ptbase = pt;
  SN10_task.UpdateXCSoarTaskFromSN10();
  return true;
}

// Simulated NMEA data stream, generated by SN10 simulator, plus comments and debug pauses
static const char * const NMEAsim[] = {
  "P rules_page",
  ";    Task Rules                    ",
  "; Task: Classic                    ",
  "; Units:Kilometers                 ",
  "; Start:Cyl     3.0                  ",
  "; Turn:FAI     0.50                  ",
  "; Finish:Line   4.0 ************     ",
  ";                                  ",
  ";                                  ",
  "P cruise_page",
  "; Avg  0.0  MC   2.0           ********",
  "; >  TEST  Water 0                 ",
  "; Out  0.0  Bugs 0%                  ",
  "; Left***  Wind360                 ",
  "; Brg ***    at  0                 ",
  "; Q 1013.3  Alt   0                  ",
  "; Cruise   Fin 607                 ",
  "; STI ***  ----730                 ",
  "P course_page",
  "; TEST                             ",
  "; 1Montpe          *******         ",
  ";                                  ",
  ";          TEST                    ",
  ";          MC:  2.0                  ",
  ";          ETF  :35                  ",
  ";          Rem 5:00                  ",
  ";          Dist45.0                  ",
  "$PILC,TSK,R,300,C,3000,,L,4000,607,F,500,*71",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,N,*50",
  "$PILC,PT,2,1Montpe,4412.233,N,07233.500,W,N,*76",
  "$PILC,PT,3*21",
  "$PILC,PT,4*26",
  "$PILC,PT,5*27",
  "$PILC,PT,6*24",
  "$PILC,PT,7*25",
  "$PILC,PT,8*2A",
  "$PILC,PT,9*2B",
  "$PILC,PT,10*13",
  "$PILC,PT,11*12",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "pause: Racing task look AOK? Should have start cyl, finish line, FAI TP.",
  "",
  "$PILC,PDA1,0,0.00*70",
  "$PILC,POLAR,-0.0015302,0.078267,-1.68293,1.0001*74",
  "$PILC,SET,1013.2*4B",
  "$PILC,TSK,R,301,C,3100,,L,4100,601,F,501,*70",
  "pause: Racing. start radius:3100, start height:blank, finish line radius:4100 height: 601. TP Radius 501?",
  "",
  "! Initial AAT setup",
  "! Task: Area, 301 minutes, Cylinder start 3000 radius, unknown start height, Line finish, 4000 meter radius, 607 finish height, FAI turn, 500 radius",
  "$PILC,TSK,A,302,C,3200,,L,4200,602,F,502,*63",
  "pause: AAT task time:302, start radius:3200, start height:blank, finish line radius:4200 height: 602. TP Radius 502?",
  "",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,N,*50",
  "pause: Spurious repeat of start point PT#1 ?",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,N,*50",
  "$PILC,PT,2*20",
  "$PILC,PT,3,2Morris,4432.183,N,07236.633,W,N,,4432.183,N,07236.633,W,5000*60",
  "$PILC,PT,5,1Montpe,4412.233,N,07233.500,W,N,,4412.328,N,07230.497,W,35000,4000,45,130*5F",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "pause: AAT task look AOK?",
  "",
  "! Dragging the target points with areas generates the following updates:",
  "$PILC,PT,3,2Morris,4432.183,N,07236.633,W,N,,4432.183,N,07237.372,W,5000*61",
  "pause: drag 2Morris",
  "$PILC,PT,3,2Morris,4432.183,N,07236.633,W,N,,4432.183,N,07238.111,W,5000*69",
  "pause: drag 2Morris",
  "$PILC,PT,5,1Montpe,4412.233,N,07233.500,W,N,,4412.328,N,07230.497,W,35000,4000,45,130*5F",
  "pause: drag 1Montpe",
  "$PILC,PT,3,2Morris,4432.183,N,07236.633,W,N,,4433.767,N,07238.111,W,5000*64",
  "pause: drag 2Morris",
  "$PILC,PT,5,1Montpe,4412.233,N,07233.500,W,N,,4412.328,N,07230.497,W,35000,4000,45,130*5F",
  "pause: drag 1Montpe",
  "$PILC,PT,5,1Montpe,4412.233,N,07233.500,W,N,,4412.328,N,07215.720,W,35000,4000,45,130*57",
  "pause: drag 1Montpe",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "pause: repeated finish?",

  "$PILC,PT,12,TEST1   ,4426.967,N,07250.000,W,N,*62",
  "pause: Moved Finish north",

  "pause: dragged points in expected locations (and not clobbered by XCSoar optimization)?",
  "",
  "! Start task, then mark TP#1 achieved",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,Y,*47",
  "pause: Check task started",
  "",
  "$PILC,PT,3,2Morris,4432.183,N,07236.633,W,Y,,4433.767,N,07238.111,W,5000*73",
  "pause: Check active leg is 2nd leg (Morrisville achieved, next TP is Montpelier)",
  "",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,Y,*47",
  "$PILC,PT,5,1Montpe,4412.233,N,07233.500,W,N,,4412.328,N,07230.497,W,36500,4000,45,130*59",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "! Return to racing task, reset to 'Before Start'",
  "$PILC,TSK,R,301,C,3000,,L,4000,607,F,500,*70",
  "pause: Check transition from AAT to racing didn't cause anything bad",
  "",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,N,*50",
  "$PILC,PT,3,AAT Pt2,4433.767,N,07238.111,W,N,*01",
  "$PILC,PT,3*21",
  "$PILC,PT,3,1Montpe,4412.233,N,07233.500,W,N,*77",
  "$PILC,PT,5,AAT Pt4,4412.328,N,07230.497,W,N,*0E",
  "$PILC,PT,5*27",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "",
  "$PILC,PT,6,2Morris,4432.183,N,07236.633,W,N,*68",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "! Start task, then achieve TP",
  "$PILC,PT,1,TEST   ,4406.967,N,07250.000,W,Y,*47",
  "$PILC,PT,3,1Montpe,4412.233,N,07233.500,W,Y,*60",
  "pause: Should show racing task, Montpelier achieved, on the way to Morrisville",
  "",
  "! Increase altitude to 3000 meters, check sqrt(sigma) in polar sentence",
  "! - Air Density Ratio (Sigma) ::= current density / standard density",
  "! - TAS = CAS / sqrt(sigma)",
  "$PILC,POLAR,-0.0012730,0.078267,-2.02283,0.8320*71",
  "",
  "! change altimeter setting to 1000.0mb",
  "$PILC,POLAR,-0.0012730,0.078267,-2.02283,0.8320*71",
  "$PILC,SET,1000.0*4B",
  "pause: verify QNH set to 1000.0"
  "",
  "! club mode task (no start point)",
  "$PILC,PT,1*23",
  "$PILC,PT,3*21",
  "$PILC,PT,6*24",
  "$PILC,PT,12,TEST   ,4406.967,N,07250.000,W,N,*62",
  "pause: Does club mode task (no start point, finish at TEST) look OK?",
  0
};

/// Simulate a series of NMEA sentences from SN10, until a 'pause' or end-of-simulation-list
static void Simulate_SN10_NMEA_sequence() {
  static int line_idx = 0;
  NMEAInfo dummyNMEAinfo = NMEAInfo(); // because, where is the "real" one? We don't need it for task input tests...
  // The ILECdevice is only instantiated in flight mode (not simulation), and only after serial port opened, so...
  static ILECDevice *dummyILECdevice = new ILECDevice();
  while(1) {
    const char* pTxt = NMEAsim[line_idx];
    if (pTxt == 0) return; // no more interesting lines
    line_idx++;
    LogFormat("%s", pTxt);
    if (memcmp(pTxt, "pause", 5) == 0)
      return; // end of sequence to simulate

    dummyILECdevice->ParseNMEA(pTxt, dummyNMEAinfo);
  };
}

#include "Input/InputEvents.hpp"
void InputEvents::eventDRNtestHook(gcc_unused const TCHAR *misc)
{ // ToDo DRN: remove test kludge
  Simulate_SN10_NMEA_sequence();
}
