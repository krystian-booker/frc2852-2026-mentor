<script setup lang="ts">
import { ref, computed } from 'vue'
import { useNTBoolean, useNTDouble, useNTInteger } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { exportCSV } from '../utils/csvExport'
import { TOPICS } from '../constants/topics'
import ConfirmModal from './ConfirmModal.vue'

const store = useCalibrationStore()

// Modal state
const showClearModal = ref(false)

// Subscribe to robot state needed for local save and validation
const hoodAtPosition = useNTBoolean(TOPICS.HOOD_AT_POSITION, false)
const flywheelAtSetpoint = useNTBoolean(TOPICS.FLYWHEEL_AT_SETPOINT, false)
const distance = useNTDouble(TOPICS.DISTANCE, 0)
const positionX = useNTDouble(TOPICS.POSITION_X, 0)
const positionY = useNTDouble(TOPICS.POSITION_Y, 0)
const gridRow = useNTInteger(TOPICS.GRID.CURRENT_ROW, 0)
const gridCol = useNTInteger(TOPICS.GRID.CURRENT_COL, 0)
const inputHoodAngle = useNTDouble(TOPICS.INPUT_HOOD_ANGLE, 25)
const inputFlywheelRPM = useNTDouble(TOPICS.INPUT_FLYWHEEL_RPM, 3000)

// Compute readyToSave locally
const readyToSave = computed(() => {
  return hoodAtPosition.value &&
         flywheelAtSetpoint.value &&
         store.isHoodAngleValid(inputHoodAngle.value) &&
         store.isFlywheelRPMValid(inputFlywheelRPM.value)
})

const exportMessage = ref('')
const importMessage = ref('')
const importIsError = ref(false)
const saveMessage = ref('')
const fileInput = ref<HTMLInputElement | null>(null)

const savePoint = () => {
  if (!readyToSave.value) return

  const isUpdate = store.hasPointAt(gridRow.value, gridCol.value)

  store.addPoint({
    robotX: positionX.value,
    robotY: positionY.value,
    distanceToTarget: distance.value,
    hoodAngleDegrees: inputHoodAngle.value,
    flywheelRPM: inputFlywheelRPM.value,
    gridRow: gridRow.value,
    gridCol: gridCol.value,
    alliance: 'Blue'
  })

  saveMessage.value = isUpdate
    ? `Updated point at [${gridRow.value}, ${gridCol.value}]`
    : `Saved point at [${gridRow.value}, ${gridCol.value}]`
  setTimeout(() => { saveMessage.value = '' }, 3000)
}

const handleExport = async () => {
  const csv = store.generateCSV()
  const success = await exportCSV(csv, 'turret_calibration_data.csv')

  if (success) {
    exportMessage.value = `Exported ${store.pointCount} points`
  } else {
    exportMessage.value = 'Export cancelled'
  }
  setTimeout(() => { exportMessage.value = '' }, 3000)
}

const triggerImport = () => {
  fileInput.value?.click()
}

const handleImport = async (event: Event) => {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file) return

  try {
    const content = await file.text()
    const result = store.importFromCSV(content)

    if (result.imported > 0) {
      importMessage.value = `Imported ${result.imported} points`
      importIsError.value = false
      if (result.errors.length > 0) {
        importMessage.value += ` (${result.errors.length} errors)`
        console.warn('CSV import errors:', result.errors)
      }
    } else if (result.errors.length > 0) {
      importMessage.value = `Import failed: ${result.errors[0]}`
      importIsError.value = true
    } else {
      importMessage.value = 'No valid data found in CSV'
      importIsError.value = true
    }

    setTimeout(() => {
      importMessage.value = ''
      importIsError.value = false
    }, 5000)
  } catch (e) {
    importMessage.value = `Import failed: ${e}`
    importIsError.value = true
    setTimeout(() => {
      importMessage.value = ''
      importIsError.value = false
    }, 5000)
  }

  input.value = ''
}

const clearData = () => {
  showClearModal.value = true
}

const confirmClear = () => {
  store.clearAllPoints()
  showClearModal.value = false
}

const cancelClear = () => {
  showClearModal.value = false
}

const deletePoint = () => {
  const row = gridRow.value
  const col = gridCol.value
  if (store.hasPointAt(row, col)) {
    store.removePoint(row, col)
  }
}
</script>

<template>
  <div class="actions-panel">
    <h3>Actions</h3>

    <div class="progress-bar">
      <div class="progress-fill" :style="{ width: store.completionPercentage + '%' }"></div>
      <span class="progress-text">
        {{ store.pointCount }} / {{ store.totalCells }} points ({{ store.completionPercentage }}%)
      </span>
    </div>

    <div class="button-row">
      <button
        class="btn btn-primary"
        :class="{ disabled: !readyToSave }"
        @click="savePoint"
        :disabled="!readyToSave"
      >
        Save Point
      </button>

      <button class="btn btn-secondary" @click="handleExport">
        Export CSV
      </button>

      <button class="btn btn-secondary" @click="triggerImport">
        Import CSV
      </button>

      <button class="btn btn-warning" @click="deletePoint">
        Delete Point
      </button>

      <button class="btn btn-danger" @click="clearData">
        Clear All
      </button>
    </div>

    <input
      ref="fileInput"
      type="file"
      accept=".csv"
      style="display: none"
      @change="handleImport"
    />

    <div v-if="saveMessage" class="message success">
      {{ saveMessage }}
    </div>

    <div v-if="exportMessage" class="message success">
      {{ exportMessage }}
    </div>

    <div v-if="importMessage" class="message" :class="{ error: importIsError }">
      {{ importMessage }}
    </div>

    <ConfirmModal
      v-if="showClearModal"
      title="Clear All Data"
      message="This will delete all calibration points. This action cannot be undone."
      confirm-text="Clear All"
      :danger="true"
      @confirm="confirmClear"
      @cancel="cancelClear"
    />
  </div>
</template>

<style scoped>
.actions-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
}

h3 {
  margin: 0 0 12px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.progress-bar {
  position: relative;
  height: 24px;
  background: #1a1a2e;
  border-radius: 4px;
  overflow: hidden;
  margin-bottom: 16px;
}

.progress-fill {
  position: absolute;
  top: 0;
  left: 0;
  height: 100%;
  background: linear-gradient(90deg, #3f51b5, #5c6bc0);
  transition: width 0.3s ease;
}

.progress-text {
  position: relative;
  z-index: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  height: 100%;
  font-size: 12px;
  font-weight: 500;
}

.button-row {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
}

.btn {
  flex: 1;
  min-width: 100px;
  padding: 12px 16px;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-primary {
  background: #4caf50;
  color: white;
}

.btn-primary:hover:not(.disabled) {
  background: #66bb6a;
}

.btn-primary.disabled {
  background: #444;
  color: #888;
  cursor: not-allowed;
}

.btn-secondary {
  background: #3f51b5;
  color: white;
}

.btn-secondary:hover {
  background: #5c6bc0;
}

.btn-warning {
  background: transparent;
  border: 1px solid #ff9800;
  color: #ff9800;
}

.btn-warning:hover {
  background: rgba(255, 152, 0, 0.1);
}

.btn-danger {
  background: transparent;
  border: 1px solid #f44336;
  color: #f44336;
}

.btn-danger:hover {
  background: rgba(244, 67, 54, 0.1);
}

.message {
  margin-top: 12px;
  padding: 8px;
  border-radius: 4px;
  font-size: 12px;
  text-align: center;
}

.message.success {
  background: rgba(76, 175, 80, 0.1);
  color: #4caf50;
}

.message.error {
  background: rgba(244, 67, 54, 0.1);
  color: #f44336;
}

.message:not(.success):not(.error) {
  background: rgba(33, 150, 243, 0.1);
  color: #2196f3;
}
</style>
