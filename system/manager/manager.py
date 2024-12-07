#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import threading
import traceback

from cereal import log
import cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.params import Params, ParamKeyType
from openpilot.common.text_window import TextWindow
from openpilot.common.conversions import Conversions as CV
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.hardware.power_monitoring import VBATT_PAUSE_CHARGING
from openpilot.system.manager.helpers import unblock_stdout, write_onroad_params, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.version import get_build_metadata, terms_version, training_version

from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.frogpilot.frogpilot_functions import convert_params, frogpilot_boot_functions, setup_frogpilot, uninstall_frogpilot
from openpilot.selfdrive.frogpilot.frogpilot_variables import frogpilot_default_params, get_frogpilot_toggles, params_memory, DEFAULT_CLASSIC_MODEL, DEFAULT_CLASSIC_MODEL_NAME


def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  setup_frogpilot(build_metadata, params)
  params_storage = Params("/persist/params")
  params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyType.DEVELOPMENT_ONLY)

  convert_params(params, params_storage)
  threading.Thread(target=frogpilot_boot_functions, args=(build_metadata, params, params_storage,)).start()

  default_params: list[tuple[str, str | bytes]] = [
    ("AlwaysOnDM", "0"),
    ("CalibrationParams", ""),
    ("CarParamsPersistent", ""),
    ("CompletedTrainingVersion", "0"),
    ("DisengageOnAccelerator", "0"),
    ("ExperimentalLongitudinalEnabled", "0"),
    ("ExperimentalMode", "0"),
    ("ExperimentalModeConfirmed", "0"),
    ("GithubSshKeys", ""),
    ("GithubUsername", ""),
    ("GsmApn", ""),
    ("GsmMetered", "1"),
    ("GsmRoaming", "1"),
    ("HasAcceptedTerms", "0"),
    ("IsLdwEnabled", "0"),
    ("IsMetric", "0"),
    ("LanguageSetting", "main_en"),
    ("LiveTorqueParameters", ""),
    ("NavSettingLeftSide", "0"),
    ("NavSettingTime24h", "0"),
    ("OpenpilotEnabledToggle", "1"),
    ("RecordFront", "0"),
    ("SshEnabled", "0"),
    ("TetheringEnabled", "0"),
    ("LongitudinalPersonality", str(log.LongitudinalPersonality.standard)),

    # Default FrogPilot parameters
    ("AccelerationPath", "1"),
    ("AccelerationProfile", "2"),
    ("AdjacentLeadsUI", "1"),
    ("AdjacentPath", "0"),
    ("AdjacentPathMetrics", "0"),
    ("AdvancedCustomUI", "0"),
    ("AdvancedLateralTune", "0"),
    ("AggressiveFollow", "1.25"),
    ("AggressiveJerkAcceleration", "50"),
    ("AggressiveJerkDanger", "100"),
    ("AggressiveJerkDeceleration", "50"),
    ("AggressiveJerkSpeed", "50"),
    ("AggressiveJerkSpeedDecrease", "50"),
    ("AggressivePersonalityProfile", "1"),
    ("AlertVolumeControl", "0"),
    ("AlwaysOnLateral", "1"),
    ("AlwaysOnLateralLKAS", "0"),
    ("AlwaysOnLateralMain", "1"),
    ("AMapKey1", ""),
    ("AMapKey2", ""),
    ("AutomaticallyUpdateModels", "0"),
    ("AutomaticUpdates", "1"),
    ("AvailableModels", ""),
    ("AvailableModelsNames", ""),
    ("BigMap", "0"),
    ("BlacklistedModels", ""),
    ("BlindSpotMetrics", "0"),
    ("BlindSpotPath", "1"),
    ("BorderMetrics", "1"),
    ("CameraView", "2"),
    ("CarMake", ""),
    ("CarModel", ""),
    ("CarModelName", ""),
    ("CECurves", "1"),
    ("CECurvesLead", "1"),
    ("CELead", "1"),
    ("CEModelStopTime", "8"),
    ("CENavigation", "1"),
    ("CENavigationIntersections", "1"),
    ("CENavigationLead", "1"),
    ("CENavigationTurns", "1"),
    ("CertifiedHerbalistDrives", "0"),
    ("CertifiedHerbalistScore", "0"),
    ("CESignalSpeed", "55"),
    ("CESignalLaneDetection", "1"),
    ("CESlowerLead", "1"),
    ("CESpeed", "0"),
    ("CESpeedLead", "0"),
    ("CEStoppedLead", "1"),
    ("ClassicModels", ""),
    ("ClusterOffset", "1.015"),
    ("Compass", "0"),
    ("ConditionalExperimental", "1"),
    ("CrosstrekTorque", "1"),
    ("CurveSensitivity", "100"),
    ("CurveSpeedControl", "0"),
    ("CustomAlerts", "1"),
    ("CustomColors", "frog"),
    ("CustomCruise", "1"),
    ("CustomCruiseLong", "5"),
    ("CustomDistanceIcons", "stock"),
    ("CustomIcons", "frog-animated"),
    ("CustomizationLevel", "0"),
    ("CustomizationLevelConfirmed", "0"),
    ("CustomPersonalities", "0"),
    ("CustomSignals", "frog"),
    ("CustomSounds", "frog"),
    ("CustomUI", "1"),
    ("DecelerationProfile", "1"),
    ("DefaultModelName", DEFAULT_CLASSIC_MODEL_NAME),
    ("DeveloperUI", "0"),
    ("DeviceManagement", "1"),
    ("DeviceShutdown", "9"),
    ("DisableOnroadUploads", "0"),
    ("DisableOpenpilotLongitudinal", "0"),
    ("DisengageVolume", "100"),
    ("DissolvedOxygenDrives", "0"),
    ("DissolvedOxygenScore", "0"),
    ("DriverCamera", "0"),
    ("DuckAmigoDrives", "0"),
    ("DuckAmigoScore", "0"),
    ("DynamicPathWidth", "0"),
    ("DynamicPedalsOnUI", "1"),
    ("EngageVolume", "100"),
    ("ExperimentalGMTune", "0"),
    ("ExperimentalModeActivation", "1"),
    ("ExperimentalModels", ""),
    ("ExperimentalModeViaDistance", "1"),
    ("ExperimentalModeViaLKAS", "1"),
    ("ExperimentalModeViaTap", "0"),
    ("Fahrenheit", "0"),
    ("ForceAutoTune", "1"),
    ("ForceAutoTuneOff", "0"),
    ("ForceFingerprint", "0"),
    ("ForceMPHDashboard", "0"),
    ("ForceStandstill", "0"),
    ("ForceStops", "0"),
    ("FPSCounter", "1"),
    ("FrogPilotToggles", ""),
    ("FrogsGoMoosTweak", "1"),
    ("FullMap", "0"),
    ("GasRegenCmd", "1"),
    ("GMapKey", ""),
    ("GoatScream", "0"),
    ("GreenLightAlert", "0"),
    ("HideAlerts", "0"),
    ("HideAOLStatusBar", "0"),
    ("HideCEMStatusBar", "0"),
    ("HideCSCUI", "0"),
    ("HideLeadMarker", "0"),
    ("HideMapIcon", "0"),
    ("HideMaxSpeed", "0"),
    ("HideSpeed", "0"),
    ("HideSpeedLimit", "0"),
    ("HolidayThemes", "1"),
    ("HumanAcceleration", "1"),
    ("HumanFollowing", "1"),
    ("IncreasedStoppedDistance", "3"),
    ("IncreaseThermalLimits", "0"),
    ("JerkInfo", "1"),
    ("LaneChangeCustomizations", "1"),
    ("LaneChangeTime", "0"),
    ("LaneDetectionWidth", "60"),
    ("LaneLinesWidth", "4"),
    ("LateralMetrics", "1"),
    ("LateralTune", "1"),
    ("LeadDepartingAlert", "0"),
    ("LeadDetectionThreshold", "35"),
    ("LeadInfo", "1"),
    ("LockDoors", "1"),
    ("LongitudinalMetrics", "1"),
    ("LongitudinalTune", "1"),
    ("LongPitch", "1"),
    ("LosAngelesDrives", "0"),
    ("LosAngelesScore", "0"),
    ("LoudBlindspotAlert", "0"),
    ("LowVoltageShutdown", str(VBATT_PAUSE_CHARGING)),
    ("MapAcceleration", "0"),
    ("MapDeceleration", "0"),
    ("MapGears", "0"),
    ("MapboxPublicKey", ""),
    ("MapboxSecretKey", ""),
    ("MapsSelected", ""),
    ("MapStyle", "10"),
    ("MaxDesiredAcceleration", "4.0"),
    ("MinimumLaneChangeSpeed", str(LANE_CHANGE_SPEED_MIN / CV.MPH_TO_MS)),
    ("Model", DEFAULT_CLASSIC_MODEL),
    ("ModelName", DEFAULT_CLASSIC_MODEL_NAME),
    ("ModelRandomizer", "0"),
    ("ModelUI", "1"),
    ("MTSCCurvatureCheck", "0"),
    ("MTSCEnabled", "1"),
    ("NavigationModels", ""),
    ("NavigationUI", "1"),
    ("NewLongAPI", "0"),
    ("NewLongAPIGM", "1"),
    ("NNFF", "1"),
    ("NNFFLite", "1"),
    ("NoLogging", "0"),
    ("NorthDakotaDrives", "0"),
    ("NorthDakotaScore", "0"),
    ("NotreDameDrives", "0"),
    ("NotreDameScore", "0"),
    ("NoUploads", "0"),
    ("NudgelessLaneChange", "1"),
    ("NumericalTemp", "1"),
    ("OfflineMode", "1"),
    ("Offset1", "5"),
    ("Offset2", "5"),
    ("Offset3", "5"),
    ("Offset4", "10"),
    ("OneLaneChange", "1"),
    ("OnroadDistanceButton", "0"),
    ("openpilotMinutes", "0"),
    ("PathEdgeWidth", "20"),
    ("PathWidth", "61"),
    ("PauseAOLOnBrake", "0"),
    ("PauseLateralOnSignal", "0"),
    ("PauseLateralSpeed", "0"),
    ("PedalsOnUI", "1"),
    ("PersonalizeOpenpilot", "1"),
    ("PreferredSchedule", "0"),
    ("PromptDistractedVolume", "100"),
    ("PromptVolume", "100"),
    ("QOLLateral", "1"),
    ("QOLLongitudinal", "1"),
    ("QOLVisuals", "1"),
    ("RadarlessModels", ""),
    ("RadicalTurtleDrives", "0"),
    ("RadicalTurtleScore", "0"),
    ("RainbowPath", "0"),
    ("RandomEvents", "0"),
    ("RecertifiedHerbalistDrives", "0"),
    ("RecertifiedHerbalistScore", "0"),
    ("RefuseVolume", "100"),
    ("RelaxedFollow", "1.75"),
    ("RelaxedJerkAcceleration", "100"),
    ("RelaxedJerkDanger", "100"),
    ("RelaxedJerkDeceleration", "100"),
    ("RelaxedJerkSpeed", "100"),
    ("RelaxedJerkSpeedDecrease", "100"),
    ("RelaxedPersonalityProfile", "1"),
    ("ResetFrogTheme", "0"),
    ("ReverseCruise", "0"),
    ("RoadEdgesWidth", "2"),
    ("RoadNameUI", "1"),
    ("RotatingWheel", "1"),
    ("ScreenBrightness", "101"),
    ("ScreenBrightnessOnroad", "101"),
    ("ScreenManagement", "1"),
    ("ScreenRecorder", "1"),
    ("ScreenTimeout", "30"),
    ("ScreenTimeoutOnroad", "30"),
    ("SearchInput", "0"),
    ("SecretGoodOpenpilotDrives", "0"),
    ("SecretGoodOpenpilotScore", "0"),
    ("SetSpeedLimit", "0"),
    ("SetSpeedOffset", "0"),
    ("ShowCPU", "1"),
    ("ShowGPU", "0"),
    ("ShowIP", "0"),
    ("ShowMemoryUsage", "1"),
    ("ShowSLCOffset", "1"),
    ("ShowSteering", "1"),
    ("ShowStoppingPoint", "0"),
    ("ShowStoppingPointMetrics", "0"),
    ("ShowStorageLeft", "0"),
    ("ShowStorageUsed", "0"),
    ("Sidebar", "0"),
    ("SidebarMetrics", "1"),
    ("SignalMetrics", "0"),
    ("SLCConfirmation", "1"),
    ("SLCConfirmationHigher", "1"),
    ("SLCConfirmationLower", "1"),
    ("SLCFallback", "2"),
    ("SLCLookaheadHigher", "5"),
    ("SLCLookaheadLower", "5"),
    ("SLCOverride", "1"),
    ("SLCPriority1", "Dashboard"),
    ("SLCPriority2", "Offline Maps"),
    ("SLCPriority3", "Navigation"),
    ("SNGHack", "1"),
    ("SpeedLimitChangedAlert", "1"),
    ("SpeedLimitController", "1"),
    ("SpeedLimitSources", "0"),
    ("StartupMessageBottom", "so I do what I want 🐸"),
    ("StartupMessageTop", "Hippity hoppity this is my property"),
    ("StandardFollow", "1.45"),
    ("StandardJerkAcceleration", "100"),
    ("StandardJerkDanger", "100"),
    ("StandardJerkDeceleration", "100"),
    ("StandardJerkSpeed", "100"),
    ("StandardJerkSpeedDecrease", "100"),
    ("StandardPersonalityProfile", "1"),
    ("StandbyMode", "0"),
    ("StaticPedalsOnUI", "0"),
    ("SteerFriction", "0.1"),
    ("SteerFrictionStock", "0.1"),
    ("SteerLatAccel", "2.5"),
    ("SteerLatAccelStock", "2.5"),
    ("SteerKP", "1"),
    ("SteerKPStock", "1"),
    ("SteerRatio", ""),
    ("SteerRatioStock", ""),
    ("StoppedTimer", "0"),
    ("TacoTune", "0"),
    ("ToyotaDoors", "0"),
    ("TrafficFollow", "0.5"),
    ("TrafficJerkAcceleration", "50"),
    ("TrafficJerkDanger", "100"),
    ("TrafficJerkDeceleration", "50"),
    ("TrafficJerkSpeed", "50"),
    ("TrafficJerkSpeedDecrease", "50"),
    ("TrafficPersonalityProfile", "1"),
    ("TuningInfo", "1"),
    ("TurnAggressiveness", "100"),
    ("TurnDesires", "0"),
    ("UnlimitedLength", "1"),
    ("UnlockDoors", "1"),
    ("UseSI", "1"),
    ("UseVienna", "0"),
    ("VisionTurnControl", "1"),
    ("VoltSNG", "0"),
    ("WarningImmediateVolume", "100"),
    ("WarningSoftVolume", "100"),
    ("WD40Drives", "0"),
    ("WD40Score", "0"),
    ("WheelIcon", "frog"),
    ("WheelSpeed", "0"),
    ("AutoNaviSpeedCtrlStart", "25"),
    ("AutoNaviSpeedCtrlEnd", "15"),
  ]
  if not PC:
    default_params.append(("LastUpdateTime", datetime.datetime.utcnow().isoformat().encode('utf8')))

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True)

  # set unset params
  reset_toggles = params.get_bool("DoToggleReset")
  for k, v in default_params + frogpilot_default_params:
    if params.get(k) is None or reset_toggles:
      if params_storage.get(k) is None or reset_toggles:
        params.put(k, v if isinstance(v, bytes) else str(v).encode('utf-8'))
      else:
        params.put(k, params_storage.get(k))
    else:
      params_storage.put(k, params.get(k))
  params.remove("DoToggleReset")

  # Create folders needed for msgq
  try:
    os.mkdir("/dev/shm")
  except FileExistsError:
    pass
  except PermissionError:
    print("WARNING: failed to make /dev/shm")

  # set version params
  params.put("Version", build_metadata.openpilot.version)
  params.put("TermsVersion", terms_version)
  params.put("TrainingVersion", training_version)
  params.put("GitCommit", build_metadata.openpilot.git_commit)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date)
  params.put("GitBranch", build_metadata.channel)
  params.put("GitRemote", build_metadata.openpilot.git_origin)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    serial = params.get("HardwareSerial")
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  sentry.init(sentry.SentryProject.SELFDRIVE)
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  # preimport all processes
  for p in managed_processes.values():
    p.prepare()


def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId", encoding='utf8') in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams', 'frogpilotPlan'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  write_onroad_params(False, params)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore, classic_model=False, frogpilot_toggles=get_frogpilot_toggles())

  started_prev = False

  # FrogPilot variables
  frogpilot_toggles = get_frogpilot_toggles()

  classic_model = frogpilot_toggles.classic_model

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_ONROAD_TRANSITION)

      # FrogPilot variables
      classic_model = frogpilot_toggles.classic_model

    elif not started and started_prev:
      params.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)
      params_memory.clear_all(ParamKeyType.CLEAR_ON_OFFROAD_TRANSITION)

    # update onroad params, which drives pandad's safety setter thread
    if started != started_prev:
      write_onroad_params(started, params)

    started_prev = started

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore, classic_model=classic_model, frogpilot_toggles=frogpilot_toggles)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}")
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break

    # Update FrogPilot parameters
    if sm['frogpilotPlan'].togglesUpdated:
      frogpilot_toggles = get_frogpilot_toggles()

def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    sentry.capture_exception()
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    uninstall_frogpilot()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
