/**
 * Centralized NetworkTables topic path constants.
 * Keeps all topic strings in one place to avoid typos and enable easy refactoring.
 */

export const NT_TABLE = '/TurretCalibration'

// Status and connection
export const TOPICS = {
  // Core state
  ENABLED: `${NT_TABLE}/Enabled`,
  STATUS: `${NT_TABLE}/Status`,
  ERROR: `${NT_TABLE}/Error`,
  LOAD_STATUS: `${NT_TABLE}/LoadStatus`,
  INFO: `${NT_TABLE}/Info`,

  // Position data
  POSITION_X: `${NT_TABLE}/Position/X`,
  POSITION_Y: `${NT_TABLE}/Position/Y`,
  DISTANCE: `${NT_TABLE}/Distance`,

  // Actual values (from robot)
  ACTUAL_HOOD_ANGLE: `${NT_TABLE}/Actual/HoodAngle`,
  ACTUAL_FLYWHEEL_RPM: `${NT_TABLE}/Actual/FlywheelRPM`,
  HOOD_AT_POSITION: `${NT_TABLE}/Actual/HoodAtPosition`,
  FLYWHEEL_AT_SETPOINT: `${NT_TABLE}/Actual/FlywheelAtSetpoint`,

  // Input values (to robot)
  INPUT_HOOD_ANGLE: `${NT_TABLE}/Input/HoodAngle`,
  INPUT_FLYWHEEL_RPM: `${NT_TABLE}/Input/FlywheelRPM`,

  // Constants (from robot)
  CONSTANTS: {
    MIN_HOOD_ANGLE: `${NT_TABLE}/Constants/MinHoodAngle`,
    MAX_HOOD_ANGLE: `${NT_TABLE}/Constants/MaxHoodAngle`,
    MIN_FLYWHEEL_RPM: `${NT_TABLE}/Constants/MinFlywheelRPM`,
    MAX_FLYWHEEL_RPM: `${NT_TABLE}/Constants/MaxFlywheelRPM`,
  },

  // Grid info
  GRID: {
    ROWS: `${NT_TABLE}/Grid/Rows`,
    COLS: `${NT_TABLE}/Grid/Cols`,
    CURRENT_ROW: `${NT_TABLE}/Grid/CurrentRow`,
    CURRENT_COL: `${NT_TABLE}/Grid/CurrentCol`,
    NEXT_TARGET_X: `${NT_TABLE}/Grid/NextTargetX`,
    NEXT_TARGET_Y: `${NT_TABLE}/Grid/NextTargetY`,
  },

  // Progress
  PROGRESS: {
    POINTS_SAVED: `${NT_TABLE}/Progress/PointsSaved`,
    TOTAL_POINTS: `${NT_TABLE}/Progress/TotalPoints`,
  },

  // Validation
  VALIDATION: {
    READY_TO_SAVE: `${NT_TABLE}/Validation/ReadyToSave`,
    WARNING: `${NT_TABLE}/Validation/Warning`,
    DISTANCE_VALID: `${NT_TABLE}/Validation/DistanceValid`,
    HOOD_ANGLE_VALID: `${NT_TABLE}/Validation/HoodAngleValid`,
    FLYWHEEL_RPM_VALID: `${NT_TABLE}/Validation/FlywheelRPMValid`,
    ALLIANCE_MISMATCH: `${NT_TABLE}/Validation/AllianceMismatch`,
    DUPLICATE_POINT: `${NT_TABLE}/Validation/DuplicatePoint`,
  },

  // Actions (triggers)
  SAVE_POINT: `${NT_TABLE}/SavePoint`,
  EXPORT: `${NT_TABLE}/Export`,
  CLEAR_DATA: `${NT_TABLE}/ClearData`,
  DELETE_CURRENT_POINT: `${NT_TABLE}/DeleteCurrentPoint`,
  DELETE_TARGET_POINT: `${NT_TABLE}/DeleteTargetPoint`,

  // Delete target coordinates
  DELETE_TARGET: {
    ROW: `${NT_TABLE}/DeleteTarget/Row`,
    COL: `${NT_TABLE}/DeleteTarget/Col`,
    SEQUENCE: `${NT_TABLE}/DeleteTarget/Sequence`,
  },

  // Data sync
  DATA: {
    POINTS: `${NT_TABLE}/Data/Points`,
    VERSION: `${NT_TABLE}/Data/Version`,
    LAST_UPDATE_TIMESTAMP: `${NT_TABLE}/Data/LastUpdateTimestamp`,
  },

  // Alliance
  ALLIANCE: {
    CURRENT: `${NT_TABLE}/Alliance/Current`,
    SELECT_OPTIONS: `${NT_TABLE}/Alliance/Select/options`,
    SELECT_SELECTED: `${NT_TABLE}/Alliance/Select/selected`,
    SELECT_ACTIVE: `${NT_TABLE}/Alliance/Select/active`,
    SELECT_DEFAULT: `${NT_TABLE}/Alliance/Select/default`,
  },
} as const

export type TopicKey = keyof typeof TOPICS
