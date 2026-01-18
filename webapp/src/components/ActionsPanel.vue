<script setup lang="ts">
import { ref } from 'vue'
import { useNTBoolean, triggerBoolean } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { exportCSV } from '../utils/csvExport'
import { TOPICS } from '../constants/topics'

const store = useCalibrationStore()

// Subscribe to validation state for save button
const readyToSave = useNTBoolean(TOPICS.VALIDATION.READY_TO_SAVE, false)

const exportMessage = ref('')
const importMessage = ref('')
const importIsError = ref(false)  // Dedicated error state instead of string matching
const fileInput = ref<HTMLInputElement | null>(null)

const savePoint = async () => {
  // Trigger robot-side save only - robot is source of truth
  // Data will sync back to webapp via NetworkTables subscription
  await triggerBoolean(TOPICS.SAVE_POINT)
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

  // Reset input so same file can be selected again
  input.value = ''
}

const clearData = async () => {
  if (confirm('Clear all calibration data? This cannot be undone.')) {
    // Trigger robot-side clear only - robot is source of truth
    // Data will sync back to webapp via NetworkTables subscription
    await triggerBoolean(TOPICS.CLEAR_DATA)
  }
}

const deletePoint = async () => {
  // Trigger robot-side delete only - robot is source of truth
  // Data will sync back to webapp via NetworkTables subscription
  await triggerBoolean(TOPICS.DELETE_CURRENT_POINT)
}

const undoMessage = ref('')

const handleUndo = () => {
  if (store.canUndo) {
    const description = store.lastUndoDescription
    const success = store.undo()
    if (success) {
      undoMessage.value = `Undone: ${description}`
      setTimeout(() => { undoMessage.value = '' }, 3000)
    }
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

      <button
        class="btn btn-undo"
        :class="{ disabled: !store.canUndo }"
        :disabled="!store.canUndo"
        @click="handleUndo"
        :title="store.canUndo ? store.lastUndoDescription : 'Nothing to undo'"
      >
        Undo
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

    <div v-if="exportMessage" class="export-message success">
      {{ exportMessage }}
    </div>

    <div v-if="importMessage" class="export-message" :class="{ error: importIsError }">
      {{ importMessage }}
    </div>

    <div v-if="undoMessage" class="export-message success">
      {{ undoMessage }}
    </div>
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

.btn-undo {
  background: transparent;
  border: 1px solid #9c27b0;
  color: #9c27b0;
}

.btn-undo:hover:not(.disabled) {
  background: rgba(156, 39, 176, 0.1);
}

.btn-undo.disabled {
  border-color: #555;
  color: #555;
  cursor: not-allowed;
}

.btn-danger {
  background: transparent;
  border: 1px solid #f44336;
  color: #f44336;
}

.btn-danger:hover {
  background: rgba(244, 67, 54, 0.1);
}

.export-message {
  margin-top: 12px;
  padding: 8px;
  border-radius: 4px;
  font-size: 12px;
  text-align: center;
}

.export-message.success {
  background: rgba(76, 175, 80, 0.1);
  color: #4caf50;
}

.export-message.error {
  background: rgba(244, 67, 54, 0.1);
  color: #f44336;
}

.export-message:not(.success):not(.error) {
  background: rgba(33, 150, 243, 0.1);
  color: #2196f3;
}
</style>
