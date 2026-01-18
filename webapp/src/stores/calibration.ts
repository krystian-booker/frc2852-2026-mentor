import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

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
const MAX_UNDO_STACK_SIZE = 10

// Undo operation type
interface UndoOperation {
  type: 'delete' | 'clear'
  points: CalibrationPoint[]
  timestamp: number
}

export const useCalibrationStore = defineStore('calibration', () => {
  // Calibration points stored in memory
  const points = ref<CalibrationPoint[]>([])

  // Undo stack for point deletions (last 10 operations)
  const undoStack = ref<UndoOperation[]>([])

  // Grid configuration (defaults, can be updated from NetworkTables)
  const gridRows = ref(13)
  const gridCols = ref(30)

  // Validation bounds (defaults, should be updated from NetworkTables constants)
  const minHoodAngle = ref(0)
  const maxHoodAngle = ref(45)
  const minFlywheelRPM = ref(1000)
  const maxFlywheelRPM = ref(6000)

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
      // Update existing point
      points.value[existingIndex] = newPoint
    } else {
      // Add new point
      points.value.push(newPoint)
    }

    saveToStorage()
  }

  // Push operation to undo stack
  const pushToUndoStack = (operation: UndoOperation) => {
    undoStack.value.push(operation)
    // Keep only last MAX_UNDO_STACK_SIZE operations
    if (undoStack.value.length > MAX_UNDO_STACK_SIZE) {
      undoStack.value.shift()
    }
  }

  // Remove a point by grid position
  const removePoint = (gridRow: number, gridCol: number) => {
    // Find the point being deleted for undo
    const deletedPoint = points.value.find(
      p => p.gridRow === gridRow && p.gridCol === gridCol
    )
    if (deletedPoint) {
      pushToUndoStack({
        type: 'delete',
        points: [deletedPoint],
        timestamp: Date.now()
      })
    }

    points.value = points.value.filter(
      p => !(p.gridRow === gridRow && p.gridCol === gridCol)
    )
    saveToStorage()
  }

  // Clear all points
  const clearAllPoints = () => {
    // Save all points for undo before clearing
    if (points.value.length > 0) {
      pushToUndoStack({
        type: 'clear',
        points: [...points.value],
        timestamp: Date.now()
      })
    }
    points.value = []
    saveToStorage()
  }

  // Undo last deletion operation
  const undo = (): boolean => {
    const operation = undoStack.value.pop()
    if (!operation) {
      return false
    }

    // Restore points
    for (const point of operation.points) {
      // Only restore if no point exists at this position
      const existingIndex = points.value.findIndex(
        p => p.gridRow === point.gridRow && p.gridCol === point.gridCol
      )
      if (existingIndex < 0) {
        points.value.push(point)
      }
    }

    saveToStorage()
    return true
  }

  // Check if undo is available
  const canUndo = computed(() => undoStack.value.length > 0)

  // Get description of last undo operation
  const lastUndoDescription = computed(() => {
    if (undoStack.value.length === 0) return ''
    const op = undoStack.value[undoStack.value.length - 1]
    if (op.type === 'clear') {
      return `Restore ${op.points.length} cleared points`
    }
    return `Restore point at [${op.points[0].gridRow}, ${op.points[0].gridCol}]`
  })

  // Check if point exists at grid position
  const hasPointAt = (gridRow: number, gridCol: number): boolean => {
    return points.value.some(p => p.gridRow === gridRow && p.gridCol === gridCol)
  }

  // Get point at grid position
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
        // Check for known header keywords (column names from CSV format)
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

        // Validate data
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

        // Validate against mechanical limits (using dynamic bounds from robot)
        if (hoodAngleDegrees < minHoodAngle.value || hoodAngleDegrees > maxHoodAngle.value) {
          errors.push(`Line ${i + 1}: Hood angle ${hoodAngleDegrees.toFixed(1)}° outside valid range (${minHoodAngle.value}-${maxHoodAngle.value}°)`)
          continue
        }

        if (flywheelRPM < minFlywheelRPM.value || flywheelRPM > maxFlywheelRPM.value) {
          errors.push(`Line ${i + 1}: Flywheel RPM ${flywheelRPM.toFixed(0)} outside valid range (${minFlywheelRPM.value}-${maxFlywheelRPM.value})`)
          continue
        }

        // Add or update point
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

  // Track if we have unsaved local changes (for future conflict detection)
  const hasLocalChanges = ref(false)

  /**
   * Syncs calibration data from robot via NetworkTables.
   * Parses the serialized CSV rows and merges with local points.
   * Robot data takes precedence, but local-only points are preserved with a warning.
   * localStorage is kept as a backup cache (write-only).
   *
   * @param serializedPoints Array of CSV row strings from robot
   * @param version Data version number from robot
   * @returns Object with sync results: { synced: number, localOnlyCount: number, conflicts: number }
   */
  const syncFromRobot = (serializedPoints: string[], version?: number): {
    synced: number
    localOnlyCount: number
    conflicts: number
  } => {
    const newPoints: CalibrationPoint[] = []

    for (const row of serializedPoints) {
      if (!row || row.trim() === '') continue

      const parts = row.split(',')
      // CSV format: timestamp,robot_x,robot_y,distance_to_target,hood_angle_degrees,flywheel_rpm,grid_row,grid_col,alliance
      if (parts.length < 8) continue

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

        // Validate parsed values
        if (isNaN(robotX) || isNaN(robotY) || isNaN(distanceToTarget) ||
            isNaN(hoodAngleDegrees) || isNaN(flywheelRPM) ||
            isNaN(gridRow) || isNaN(gridCol)) {
          console.warn('Skipping invalid point from robot:', row)
          continue
        }

        newPoints.push({
          timestamp,
          robotX,
          robotY,
          distanceToTarget,
          hoodAngleDegrees,
          flywheelRPM,
          gridRow,
          gridCol,
          alliance
        })
      } catch (e) {
        console.warn('Error parsing point from robot:', row, e)
      }
    }

    // Detect conflicts: local points at positions that robot also has (with different data)
    let conflicts = 0
    const robotPositions = new Set(newPoints.map(p => `${p.gridRow},${p.gridCol}`))
    const localOnlyPoints: CalibrationPoint[] = []

    for (const localPoint of points.value) {
      const key = `${localPoint.gridRow},${localPoint.gridCol}`
      if (!robotPositions.has(key)) {
        // Local-only point - robot doesn't have it
        localOnlyPoints.push(localPoint)
      } else {
        // Check if data differs (potential conflict)
        const robotPoint = newPoints.find(p => p.gridRow === localPoint.gridRow && p.gridCol === localPoint.gridCol)
        if (robotPoint &&
            (Math.abs(robotPoint.hoodAngleDegrees - localPoint.hoodAngleDegrees) > 0.1 ||
             Math.abs(robotPoint.flywheelRPM - localPoint.flywheelRPM) > 1)) {
          conflicts++
        }
      }
    }

    // Log conflict/local-only warnings
    if (localOnlyPoints.length > 0) {
      console.warn(`Sync warning: ${localOnlyPoints.length} local-only points will be overwritten by robot data`)
    }
    if (conflicts > 0) {
      console.warn(`Sync warning: ${conflicts} points have conflicting values (robot data will be used)`)
    }

    // Replace local points with robot data (robot is source of truth)
    points.value = newPoints
    hasLocalChanges.value = false
    // Version parameter reserved for future conflict detection
    void version

    // Save to localStorage as backup cache
    saveToStorage()

    return {
      synced: newPoints.length,
      localOnlyCount: localOnlyPoints.length,
      conflicts
    }
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
    clearAllPoints,
    hasPointAt,
    getPointAt,
    totalCells,
    pointCount,
    completionPercentage,
    generateCSV,
    importFromCSV,
    loadFromStorage,
    saveToStorage,
    setGridDimensions,
    setValidationBounds,
    syncFromRobot,
    // Undo functionality
    undo,
    canUndo,
    lastUndoDescription
  }
})
