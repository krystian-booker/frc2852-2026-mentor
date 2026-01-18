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

export const useCalibrationStore = defineStore('calibration', () => {
  // Calibration points stored in memory
  const points = ref<CalibrationPoint[]>([])

  // Grid configuration (should match Java constants)
  const gridRows = 8
  const gridCols = 17

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

  // Remove a point by grid position
  const removePoint = (gridRow: number, gridCol: number) => {
    points.value = points.value.filter(
      p => !(p.gridRow === gridRow && p.gridCol === gridCol)
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

  // Get point at grid position
  const getPointAt = (gridRow: number, gridCol: number): CalibrationPoint | undefined => {
    return points.value.find(p => p.gridRow === gridRow && p.gridCol === gridCol)
  }

  // Computed: total grid cells
  const totalCells = computed(() => gridRows * gridCols)

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

  // Initialize
  loadFromStorage()

  return {
    points,
    gridRows,
    gridCols,
    addPoint,
    removePoint,
    clearAllPoints,
    hasPointAt,
    getPointAt,
    totalCells,
    pointCount,
    completionPercentage,
    generateCSV,
    loadFromStorage,
    saveToStorage
  }
})
