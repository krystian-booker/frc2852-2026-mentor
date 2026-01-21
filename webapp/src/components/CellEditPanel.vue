<script setup lang="ts">
import { ref, computed, watch } from 'vue'
import { useCalibrationStore } from '../stores/calibration'
import { CALIBRATION_AREA } from '../constants/fieldConstants'

const props = defineProps<{
  row: number
  col: number
}>()

const emit = defineEmits<{
  (e: 'close'): void
}>()

const store = useCalibrationStore()

// Local editable values
const hoodAngle = ref(25)
const flywheelRPM = ref(3000)

// Calculate field coordinates for this cell
const cellX = computed(() => CALIBRATION_AREA.minX + props.col * CALIBRATION_AREA.cellSize)
const cellY = computed(() => CALIBRATION_AREA.minY + props.row * CALIBRATION_AREA.cellSize)

// Check if point exists at this cell
const existingPoint = computed(() => {
  return store.points.find(p => p.gridRow === props.row && p.gridCol === props.col)
})

const hasPoint = computed(() => !!existingPoint.value)

// Load existing values when cell changes or point exists
watch([() => props.row, () => props.col], () => {
  if (existingPoint.value) {
    hoodAngle.value = existingPoint.value.hoodAngleDegrees
    flywheelRPM.value = existingPoint.value.flywheelRPM
  } else {
    // Default values for new points
    hoodAngle.value = 25
    flywheelRPM.value = 3000
  }
}, { immediate: true })

// Validation
const isHoodAngleValid = computed(() => store.isHoodAngleValid(hoodAngle.value))
const isFlywheelRPMValid = computed(() => store.isFlywheelRPMValid(flywheelRPM.value))
const canSave = computed(() => isHoodAngleValid.value && isFlywheelRPMValid.value)

// Message state
const message = ref('')
const messageType = ref<'success' | 'error'>('success')

const showMessage = (text: string, type: 'success' | 'error' = 'success') => {
  message.value = text
  messageType.value = type
  setTimeout(() => { message.value = '' }, 3000)
}

const savePoint = () => {
  if (!canSave.value) return

  const isUpdate = hasPoint.value

  store.addPoint({
    robotX: cellX.value,
    robotY: cellY.value,
    distanceToTarget: Math.sqrt(cellX.value ** 2 + cellY.value ** 2), // Approximate
    hoodAngleDegrees: hoodAngle.value,
    flywheelRPM: flywheelRPM.value,
    gridRow: props.row,
    gridCol: props.col,
    alliance: 'Blue'
  })

  showMessage(isUpdate ? 'Point updated' : 'Point saved', 'success')
}

const deletePoint = () => {
  if (hasPoint.value) {
    store.removePoint(props.row, props.col)
    showMessage('Point deleted', 'success')
  }
}

const close = () => {
  emit('close')
}
</script>

<template>
  <div class="cell-edit-panel">
    <div class="panel-header">
      <h3>Edit Cell [{{ row }}, {{ col }}]</h3>
      <button class="close-btn" @click="close">&times;</button>
    </div>

    <div class="cell-info">
      <span class="info-item">
        <span class="label">Position</span>
        <span class="value">({{ cellX.toFixed(1) }}m, {{ cellY.toFixed(1) }}m)</span>
      </span>
      <span class="info-item" :class="{ 'has-data': hasPoint }">
        <span class="label">Status</span>
        <span class="value">{{ hasPoint ? 'Calibrated' : 'Empty' }}</span>
      </span>
    </div>

    <div class="input-group">
      <label for="hood-angle">Hood Angle (deg)</label>
      <input
        id="hood-angle"
        type="number"
        v-model.number="hoodAngle"
        :min="store.minHoodAngle"
        :max="store.maxHoodAngle"
        step="0.5"
        :class="{ invalid: !isHoodAngleValid }"
      />
      <span class="bounds">{{ store.minHoodAngle }} - {{ store.maxHoodAngle }}°</span>
    </div>

    <div class="input-group">
      <label for="flywheel-rpm">Flywheel RPM</label>
      <input
        id="flywheel-rpm"
        type="number"
        v-model.number="flywheelRPM"
        :min="store.minFlywheelRPM"
        :max="store.maxFlywheelRPM"
        step="50"
        :class="{ invalid: !isFlywheelRPMValid }"
      />
      <span class="bounds">{{ store.minFlywheelRPM }} - {{ store.maxFlywheelRPM }} RPM</span>
    </div>

    <div class="button-row">
      <button
        class="btn btn-primary"
        :class="{ disabled: !canSave }"
        :disabled="!canSave"
        @click="savePoint"
      >
        {{ hasPoint ? 'Update' : 'Save' }}
      </button>
      <button
        v-if="hasPoint"
        class="btn btn-danger"
        @click="deletePoint"
      >
        Delete
      </button>
    </div>

    <div v-if="message" class="message" :class="messageType">
      {{ message }}
    </div>
  </div>
</template>

<style scoped>
.cell-edit-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
  border: 1px solid #3f51b5;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

h3 {
  margin: 0;
  font-size: 14px;
  color: #2196f3;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.close-btn {
  background: transparent;
  border: none;
  color: #888;
  font-size: 24px;
  cursor: pointer;
  padding: 0;
  line-height: 1;
}

.close-btn:hover {
  color: #fff;
}

.cell-info {
  display: flex;
  gap: 16px;
  margin-bottom: 16px;
  padding: 10px;
  background: #1a1a2e;
  border-radius: 6px;
}

.info-item {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.info-item .label {
  font-size: 11px;
  color: #888;
  text-transform: uppercase;
}

.info-item .value {
  font-size: 14px;
  font-family: 'Consolas', monospace;
}

.info-item.has-data .value {
  color: #4caf50;
}

.input-group {
  margin-bottom: 12px;
}

.input-group label {
  display: block;
  font-size: 12px;
  color: #888;
  margin-bottom: 4px;
}

.input-group input {
  width: 100%;
  padding: 10px;
  background: #1a1a2e;
  border: 1px solid #444;
  border-radius: 4px;
  color: #fff;
  font-size: 16px;
  font-family: 'Consolas', monospace;
}

.input-group input:focus {
  outline: none;
  border-color: #3f51b5;
}

.input-group input.invalid {
  border-color: #f44336;
  background: rgba(244, 67, 54, 0.1);
}

.input-group .bounds {
  display: block;
  font-size: 11px;
  color: #666;
  margin-top: 4px;
}

.button-row {
  display: flex;
  gap: 12px;
  margin-top: 16px;
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
</style>
