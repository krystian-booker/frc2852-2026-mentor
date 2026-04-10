/**
 * Centralized NetworkTables topic path constants.
 * Keeps all topic strings in one place to avoid typos and enable easy refactoring.
 */

export const NT_TABLE = '/TurretCalibration'

export const TOPICS = {
  // Core state
  ENABLED: `${NT_TABLE}/Enabled`,
  STATUS: `${NT_TABLE}/Status`,
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
  OBSERVED_INPUT_HOOD_ANGLE: `${NT_TABLE}/ObservedInput/HoodAngle`,
  OBSERVED_INPUT_FLYWHEEL_RPM: `${NT_TABLE}/ObservedInput/FlywheelRPM`,

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
} as const

export type TopicKey = keyof typeof TOPICS
