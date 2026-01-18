<script setup lang="ts">
import { ref } from 'vue'
import { useNTDouble, useNTInteger, useNTBoolean, triggerBoolean } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { exportCSV } from '../utils/csvExport'

const store = useCalibrationStore()

// Read current values for saving
const posX = useNTDouble('/TurretCalibration/Position/X', 0)
const posY = useNTDouble('/TurretCalibration/Position/Y', 0)
const distance = useNTDouble('/TurretCalibration/Distance', 0)
const inputHoodAngle = useNTDouble('/TurretCalibration/Input/HoodAngle', 25)
const inputFlywheelRPM = useNTDouble('/TurretCalibration/Input/FlywheelRPM', 3000)
const gridRow = useNTInteger('/TurretCalibration/Grid/CurrentRow', 0)
const gridCol = useNTInteger('/TurretCalibration/Grid/CurrentCol', 0)
const readyToSave = useNTBoolean('/TurretCalibration/Validation/ReadyToSave', false)

const exportMessage = ref('')

const savePoint = () => {
  // Save to local store
  store.addPoint({
    robotX: posX.value,
    robotY: posY.value,
    distanceToTarget: distance.value,
    hoodAngleDegrees: inputHoodAngle.value,
    flywheelRPM: inputFlywheelRPM.value,
    gridRow: gridRow.value,
    gridCol: gridCol.value,
    alliance: 'Blue' // Could be made configurable
  })

  // Also trigger robot-side save
  triggerBoolean('/TurretCalibration/SavePoint')
}

const handleExport = async () => {
  const csv = store.generateCSV()
  const success = await exportCSV(csv, 'turret_calibration_data.csv')

  if (success) {
    exportMessage.value = `Exported ${store.pointCount} points`
    setTimeout(() => { exportMessage.value = '' }, 3000)
  }
}

const clearData = () => {
  if (confirm('Clear all calibration data? This cannot be undone.')) {
    store.clearAllPoints()
    triggerBoolean('/TurretCalibration/ClearData')
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

      <button class="btn btn-danger" @click="clearData">
        Clear All
      </button>
    </div>

    <div v-if="exportMessage" class="export-message">
      {{ exportMessage }}
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
}

.btn {
  flex: 1;
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
  background: rgba(76, 175, 80, 0.1);
  color: #4caf50;
  font-size: 12px;
  text-align: center;
}
</style>
