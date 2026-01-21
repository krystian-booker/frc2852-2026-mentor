// Field image dimensions and coordinates for 2026 FRC field

export const FIELD_IMAGE = {
  path: '/field/2026-field.png',
  width: 4196,
  height: 2035,
}

// Playable area pixel coordinates within the image
export const PLAYABLE_AREA = {
  topLeft: { x: 245, y: 118 },
  bottomRight: { x: 3942, y: 1914 },
  width: 3942 - 245, // 3697 pixels
  height: 1914 - 118, // 1796 pixels
}

// Field dimensions in meters
export const FIELD_DIMENSIONS = {
  widthMeters: 16.54,
  heightMeters: 8.07,
}

// Calibration area bounds in meters
export const CALIBRATION_AREA = {
  minX: 0,
  maxX: FIELD_DIMENSIONS.widthMeters,
  minY: 0,
  maxY: FIELD_DIMENSIONS.heightMeters,
  cellSize: 0.5,
}

// Computed values
export const PIXELS_PER_METER = {
  x: PLAYABLE_AREA.width / FIELD_DIMENSIONS.widthMeters, // ~223.5
  y: PLAYABLE_AREA.height / FIELD_DIMENSIONS.heightMeters, // ~222.5
}

// Grid dimensions (calculated from field size and cell size)
export const GRID = {
  rows: Math.ceil(FIELD_DIMENSIONS.heightMeters / CALIBRATION_AREA.cellSize), // 17
  cols: Math.ceil(FIELD_DIMENSIONS.widthMeters / CALIBRATION_AREA.cellSize),  // 34
}
