import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { FIELD_DIMENSIONS, CALIBRATION_AREA } from '../constants/fieldConstants'

export interface CalibrationPoint {
  timestamp: string
  robotX: number
  robotY: number
  distanceToTarget: number
  hoodAngleDegrees: number
  flywheelRPM: number
  gridRow: number
  gridCol: number
  alliance: string
}

const STORAGE_KEY = 'turret-calibration-points'

export const useCalibrationStore = defineStore('calibration', () => {
  // Calibration points stored in memory
  const points = ref<CalibrationPoint[]>([])

  // Grid configuration (defaults, can be updated from NetworkTables)
  // Full field: 16.54m x 8.07m with 0.5m cells = 34 cols x 17 rows
  const gridRows = ref(17)
  const gridCols = ref(34)

  // Validation bounds (defaults, should be updated from NetworkTables constants)
  const minHoodAngle = ref(0)
  const maxHoodAngle = ref(25)
  const minFlywheelRPM = ref(1000)
  const maxFlywheelRPM = ref(4700)

  // Load from localStorage on init
  const loadFromStorage = () => {
    try {
      const stored = localStorage.getItem(STORAGE_KEY)
      if (stored) {
        points.value = JSON.parse(stored)
      }
    } catch (e) {
      console.error('Failed to load calibration data from storage:', e)
    }
  }

  // Save to localStorage
  const saveToStorage = () => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(points.value))
    } catch (e) {
      console.error('Failed to save calibration data to storage:', e)
    }
  }

  // Maximum grid dimensions to prevent memory issues
  const MAX_GRID_ROWS = 100
  const MAX_GRID_COLS = 100

  // Update grid dimensions (called when NT values change)
  const setGridDimensions = (rows: number, cols: number) => {
    if (rows > 0 && rows <= MAX_GRID_ROWS) {
      gridRows.value = rows
    } else if (rows > MAX_GRID_ROWS) {
      console.warn(`Grid rows ${rows} exceeds max ${MAX_GRID_ROWS}, clamping`)
      gridRows.value = MAX_GRID_ROWS
    }
    if (cols > 0 && cols <= MAX_GRID_COLS) {
      gridCols.value = cols
    } else if (cols > MAX_GRID_COLS) {
      console.warn(`Grid cols ${cols} exceeds max ${MAX_GRID_COLS}, clamping`)
      gridCols.value = MAX_GRID_COLS
    }
  }

  // Update validation bounds (called when NT constants are received)
  const setValidationBounds = (
    minHood: number,
    maxHood: number,
    minRPM: number,
    maxRPM: number
  ) => {
    minHoodAngle.value = minHood
    maxHoodAngle.value = maxHood
    minFlywheelRPM.value = minRPM
    maxFlywheelRPM.value = maxRPM
  }

  // Local validation helpers
  const isHoodAngleValid = (angle: number): boolean => {
    return angle >= minHoodAngle.value && angle <= maxHoodAngle.value
  }

  const isFlywheelRPMValid = (rpm: number): boolean => {
    return rpm >= minFlywheelRPM.value && rpm <= maxFlywheelRPM.value
  }

  // Add a calibration point
  const addPoint = (point: Omit<CalibrationPoint, 'timestamp'>) => {
    const newPoint: CalibrationPoint = {
      ...point,
      timestamp: new Date().toISOString()
    }

    // Check if point exists at this grid position
    const existingIndex = points.value.findIndex(
      p => p.gridRow === point.gridRow && p.gridCol === point.gridCol
    )

    if (existingIndex >= 0) {
      points.value[existingIndex] = newPoint
    } else {
      points.value.push(newPoint)
    }

    saveToStorage()
  }

  // Remove a point by grid position
  const removePoint = (gridRow: number, gridCol: number) => {
    points.value = points.value.filter(
      p => !(p.gridRow === gridRow && p.gridCol === gridCol)
    )
    saveToStorage()
  }

  // Bulk update existing points and create new ones for empty cells
  const bulkUpdatePoints = (
    positions: { row: number; col: number }[],
    updates: { hoodAngleDegrees: number; flywheelRPM: number }
  ) => {
    const posSet = new Set(positions.map(p => `${p.row},${p.col}`))
    const existingKeys = new Set<string>()

    // Update existing points
    for (const point of points.value) {
      const key = `${point.gridRow},${point.gridCol}`
      if (posSet.has(key)) {
        point.hoodAngleDegrees = updates.hoodAngleDegrees
        point.flywheelRPM = updates.flywheelRPM
        point.timestamp = new Date().toISOString()
        existingKeys.add(key)
      }
    }

    // Create new points for empty cells
    for (const pos of positions) {
      const key = `${pos.row},${pos.col}`
      if (!existingKeys.has(key)) {
        const robotX = CALIBRATION_AREA.minX + pos.col * CALIBRATION_AREA.cellSize
        const robotY = CALIBRATION_AREA.minY + pos.row * CALIBRATION_AREA.cellSize
        points.value.push({
          timestamp: new Date().toISOString(),
          robotX,
          robotY,
          distanceToTarget: Math.sqrt(robotX ** 2 + robotY ** 2),
          hoodAngleDegrees: updates.hoodAngleDegrees,
          flywheelRPM: updates.flywheelRPM,
          gridRow: pos.row,
          gridCol: pos.col,
          alliance: 'Blue'
        })
      }
    }

    saveToStorage()
  }

  // Bulk remove points by grid positions
  const bulkRemovePoints = (positions: { row: number; col: number }[]) => {
    const posSet = new Set(positions.map(p => `${p.row},${p.col}`))
    points.value = points.value.filter(
      p => !posSet.has(`${p.gridRow},${p.gridCol}`)
    )
    saveToStorage()
  }

  // Clear all points
  const clearAllPoints = () => {
    points.value = []
    saveToStorage()
  }

  // Check if point exists at grid position
  const hasPointAt = (gridRow: number, gridCol: number): boolean => {
    return points.value.some(p => p.gridRow === gridRow && p.gridCol === gridCol)
  }

  const getPointAt = (gridRow: number, gridCol: number): CalibrationPoint | undefined => {
    return points.value.find(p => p.gridRow === gridRow && p.gridCol === gridCol)
  }

  // Computed: total grid cells
  const totalCells = computed(() => gridRows.value * gridCols.value)

  // Computed: number of saved points
  const pointCount = computed(() => points.value.length)

  // Computed: completion percentage
  const completionPercentage = computed(() =>
    Math.round((points.value.length / totalCells.value) * 100)
  )

  // Mirror points from bottom half to top half across the entire field width
  const mirrorBottomToTop = (): { copied: number } => {
    const halfRows = Math.floor(gridRows.value / 2)

    // Get all points in the bottom half (low rows, any column)
    const sourcePoints = points.value.filter(
      p => p.gridRow < halfRows
    )

    let copied = 0

    for (const source of sourcePoints) {
      const targetRow = gridRows.value - 1 - source.gridRow

      addPoint({
        robotX: source.robotX,
        robotY: FIELD_DIMENSIONS.heightMeters - source.robotY,
        distanceToTarget: source.distanceToTarget,
        hoodAngleDegrees: source.hoodAngleDegrees,
        flywheelRPM: source.flywheelRPM,
        gridRow: targetRow,
        gridCol: source.gridCol,
        alliance: source.alliance
      })
      copied++
    }

    return { copied }
  }

  // Generate CSV content
  const generateCSV = (): string => {
    const header = 'timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,flywheel_rpm,grid_row,grid_col,alliance'
    const rows = points.value.map(p =>
      `${p.timestamp},${p.robotX.toFixed(3)},${p.robotY.toFixed(3)},${p.distanceToTarget.toFixed(3)},${p.hoodAngleDegrees.toFixed(2)},${p.flywheelRPM.toFixed(0)},${p.gridRow},${p.gridCol},${p.alliance}`
    )
    return [header, ...rows].join('\n')
  }

  // Import from CSV content
  const importFromCSV = (csvContent: string): { imported: number; errors: string[] } => {
    const lines = csvContent.trim().split('\n')
    const errors: string[] = []
    let imported = 0
    let headerSkipped = false

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i].trim()
      if (!line) continue

      // Skip header row - detect by checking for known header field names
      if (!headerSkipped) {
        headerSkipped = true
        const lowerLine = line.toLowerCase()
        const headerKeywords = ['timestamp', 'robot_x', 'robot_y', 'distance_to_target', 'hood_angle', 'flywheel_rpm', 'grid_row', 'grid_col']
        const isHeader = headerKeywords.some(keyword => lowerLine.includes(keyword))
        if (isHeader) {
          continue
        }
      }

      const parts = line.split(',')
      if (parts.length < 8) {
        errors.push(`Line ${i + 1}: Not enough columns (expected 8+, got ${parts.length})`)
        continue
      }

      try {
        const timestamp = parts[0].trim()
        const robotX = parseFloat(parts[1].trim())
        const robotY = parseFloat(parts[2].trim())
        const distanceToTarget = parseFloat(parts[3].trim())
        const hoodAngleDegrees = parseFloat(parts[4].trim())
        const flywheelRPM = parseFloat(parts[5].trim())
        const gridRow = parseInt(parts[6].trim(), 10)
        const gridCol = parseInt(parts[7].trim(), 10)
        const alliance = parts.length >= 9 ? parts[8].trim() || 'Unknown' : 'Unknown'

        if (isNaN(robotX) || isNaN(robotY) || isNaN(distanceToTarget) ||
            isNaN(hoodAngleDegrees) || isNaN(flywheelRPM) ||
            isNaN(gridRow) || isNaN(gridCol)) {
          errors.push(`Line ${i + 1}: Invalid numeric values`)
          continue
        }

        if (distanceToTarget <= 0) {
          errors.push(`Line ${i + 1}: Invalid distance (must be > 0)`)
          continue
        }

        if (hoodAngleDegrees < minHoodAngle.value || hoodAngleDegrees > maxHoodAngle.value) {
          errors.push(`Line ${i + 1}: Hood angle ${hoodAngleDegrees.toFixed(1)}° outside valid range (${minHoodAngle.value}-${maxHoodAngle.value}°)`)
          continue
        }

        if (flywheelRPM < minFlywheelRPM.value || flywheelRPM > maxFlywheelRPM.value) {
          errors.push(`Line ${i + 1}: Flywheel RPM ${flywheelRPM.toFixed(0)} outside valid range (${minFlywheelRPM.value}-${maxFlywheelRPM.value})`)
          continue
        }

        const existingIndex = points.value.findIndex(
          p => p.gridRow === gridRow && p.gridCol === gridCol
        )

        const point: CalibrationPoint = {
          timestamp,
          robotX,
          robotY,
          distanceToTarget,
          hoodAngleDegrees,
          flywheelRPM,
          gridRow,
          gridCol,
          alliance
        }

        if (existingIndex >= 0) {
          points.value[existingIndex] = point
        } else {
          points.value.push(point)
        }
        imported++
      } catch (e) {
        errors.push(`Line ${i + 1}: Parse error - ${e}`)
      }
    }

    if (imported > 0) {
      saveToStorage()
    }

    return { imported, errors }
  }

  // Initialize
  loadFromStorage()

  return {
    points,
    gridRows,
    gridCols,
    minHoodAngle,
    maxHoodAngle,
    minFlywheelRPM,
    maxFlywheelRPM,
    addPoint,
    removePoint,
    bulkUpdatePoints,
    bulkRemovePoints,
    clearAllPoints,
    hasPointAt,
    getPointAt,
    totalCells,
    pointCount,
    completionPercentage,
    generateCSV,
    importFromCSV,
    setGridDimensions,
    setValidationBounds,
    isHoodAngleValid,
    isFlywheelRPMValid,
    mirrorBottomToTop
  }
})
