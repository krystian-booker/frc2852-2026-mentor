<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch, computed } from 'vue'
import { useNTDouble, useNTInteger, useNTBoolean } from './composables/useNetworkTables'
import { useCalibrationStore } from './stores/calibration'
import { exportCSV } from './utils/csvExport'
import { TOPICS } from './constants/topics'
import StatusPanel from './components/StatusPanel.vue'
import PositionPanel from './components/PositionPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import ValidationPanel from './components/ValidationPanel.vue'
import ActionsPanel from './components/ActionsPanel.vue'
import DataTable from './components/DataTable.vue'
import CalibrationGrid from './components/CalibrationGrid.vue'
import CellEditPanel from './components/CellEditPanel.vue'

const store = useCalibrationStore()

// Selected cell state for manual editing
const selectedRow = ref<number | undefined>(undefined)
const selectedCol = ref<number | undefined>(undefined)
const hasSelectedCell = computed(() => selectedRow.value !== undefined && selectedCol.value !== undefined)

const handleCellSelect = (row: number, col: number) => {
  // Toggle selection if clicking the same cell
  if (selectedRow.value === row && selectedCol.value === col) {
    selectedRow.value = undefined
    selectedCol.value = undefined
  } else {
    selectedRow.value = row
    selectedCol.value = col
  }
}

const handleCellEditClose = () => {
  selectedRow.value = undefined
  selectedCol.value = undefined
}

// Subscribe to robot state needed for local save
const positionX = useNTDouble(TOPICS.POSITION_X, 0)
const positionY = useNTDouble(TOPICS.POSITION_Y, 0)
const distance = useNTDouble(TOPICS.DISTANCE, 0)
const gridRow = useNTInteger(TOPICS.GRID.CURRENT_ROW, 0)
const gridCol = useNTInteger(TOPICS.GRID.CURRENT_COL, 0)
const hoodAtPosition = useNTBoolean(TOPICS.HOOD_AT_POSITION, false)
const flywheelAtSetpoint = useNTBoolean(TOPICS.FLYWHEEL_AT_SETPOINT, false)
const inputHoodAngle = useNTDouble(TOPICS.INPUT_HOOD_ANGLE, 25)
const inputFlywheelRPM = useNTDouble(TOPICS.INPUT_FLYWHEEL_RPM, 3000)

// Subscribe to robot constants for validation bounds
const minHoodAngle = useNTDouble(TOPICS.CONSTANTS.MIN_HOOD_ANGLE, 0)
const maxHoodAngle = useNTDouble(TOPICS.CONSTANTS.MAX_HOOD_ANGLE, 25)
const minFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MIN_FLYWHEEL_RPM, 500)
const maxFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MAX_FLYWHEEL_RPM, 4700)

// Watch for constants changes and update store validation bounds
watch(
  [minHoodAngle, maxHoodAngle, minFlywheelRPM, maxFlywheelRPM],
  ([minHood, maxHood, minRPM, maxRPM]) => {
    store.setValidationBounds(minHood, maxHood, minRPM, maxRPM)
  },
  { immediate: true }
)

// Compute local validation for save
const canSaveCurrentPoint = computed(() => {
  return hoodAtPosition.value &&
         flywheelAtSetpoint.value &&
         store.isHoodAngleValid(inputHoodAngle.value) &&
         store.isFlywheelRPMValid(inputFlywheelRPM.value)
})

// Local save function
const saveCurrentPoint = () => {
  if (!canSaveCurrentPoint.value) return

  store.addPoint({
    robotX: positionX.value,
    robotY: positionY.value,
    distanceToTarget: distance.value,
    hoodAngleDegrees: inputHoodAngle.value,
    flywheelRPM: inputFlywheelRPM.value,
    gridRow: gridRow.value,
    gridCol: gridCol.value,
    alliance: 'Blue'  // Default alliance
  })
}

const handleKeydown = async (event: KeyboardEvent) => {
  // Check for Ctrl (or Cmd on Mac) key
  const isModified = event.ctrlKey || event.metaKey

  if (isModified) {
    switch (event.key.toLowerCase()) {
      case 's':
        // Ctrl+S = Save Point locally
        event.preventDefault()
        if (canSaveCurrentPoint.value) {
          saveCurrentPoint()
        }
        break

      case 'e':
        // Ctrl+E = Export CSV
        event.preventDefault()
        const csv = store.generateCSV()
        exportCSV(csv, 'turret_calibration_data.csv')
        break
    }
  }
}

onMounted(() => {
  window.addEventListener('keydown', handleKeydown)
})

onUnmounted(() => {
  window.removeEventListener('keydown', handleKeydown)
})
</script>

<template>
  <div class="app">
    <header class="header">
      <h1>Turret Calibration</h1>
      <span class="subtitle">FRC 2852 - 2026</span>
      <span class="shortcuts">Ctrl+S: Save | Ctrl+E: Export</span>
    </header>

    <main class="main-layout">
      <div class="left-column">
        <StatusPanel />
        <PositionPanel />
        <ValidationPanel />
        <ActionsPanel />
      </div>

      <div class="center-column">
        <ControlPanel />
        <CalibrationGrid
          :selected-row="selectedRow"
          :selected-col="selectedCol"
          @cell-select="handleCellSelect"
        />
        <CellEditPanel
          v-if="hasSelectedCell"
          :row="selectedRow!"
          :col="selectedCol!"
          @close="handleCellEditClose"
        />
      </div>

      <div class="right-column">
        <DataTable />
      </div>
    </main>

    <footer class="footer">
      <span>Connect to robot at port 5810 (NetworkTables)</span>
      <span>|</span>
      <span>Click grid cells to edit manually</span>
      <span>|</span>
      <span>Data stored locally in browser</span>
    </footer>
  </div>
</template>

<style>
.app {
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  padding: 20px;
  max-width: 1600px;
  margin: 0 auto;
}

.header {
  display: flex;
  align-items: baseline;
  gap: 16px;
  margin-bottom: 20px;
  padding-bottom: 16px;
  border-bottom: 1px solid #333;
}

.header h1 {
  margin: 0;
  font-size: 24px;
  font-weight: 600;
}

.subtitle {
  color: #666;
  font-size: 14px;
}

.shortcuts {
  margin-left: auto;
  color: #666;
  font-size: 12px;
  background: #252540;
  padding: 4px 12px;
  border-radius: 4px;
}

.main-layout {
  display: grid;
  grid-template-columns: 320px 1fr 360px;
  gap: 20px;
  flex: 1;
}

.left-column,
.center-column,
.right-column {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.right-column {
  display: flex;
  flex-direction: column;
}

.right-column > * {
  flex: 1;
  min-height: 0;
}

.footer {
  margin-top: 20px;
  padding-top: 16px;
  border-top: 1px solid #333;
  display: flex;
  justify-content: center;
  gap: 12px;
  color: #666;
  font-size: 12px;
}

@media (max-width: 1200px) {
  .main-layout {
    grid-template-columns: 1fr 1fr;
  }

  .right-column {
    grid-column: span 2;
  }
}

@media (max-width: 768px) {
  .main-layout {
    grid-template-columns: 1fr;
  }

  .right-column {
    grid-column: span 1;
  }

  .shortcuts {
    display: none;
  }
}
</style>
