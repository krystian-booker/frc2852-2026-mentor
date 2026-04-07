<script setup lang="ts">
import { ref, computed } from 'vue'
import { useCalibrationStore } from '../stores/calibration'
import ConfirmModal from './ConfirmModal.vue'

const props = defineProps<{
  selectedPositions: { row: number; col: number }[]
}>()

const emit = defineEmits<{
  (e: 'close'): void
}>()

const store = useCalibrationStore()

// Bulk edit fields
const hoodAngle = ref(25)
const flywheelRPM = ref(3000)

const isHoodValid = computed(() => store.isHoodAngleValid(hoodAngle.value))
const isRPMValid = computed(() => store.isFlywheelRPMValid(flywheelRPM.value))
const canApply = computed(() => isHoodValid.value && isRPMValid.value)

// Count how many of the selected cells actually have data
const selectedWithData = computed(() =>
  props.selectedPositions.filter(p => store.hasPointAt(p.row, p.col)).length
)
const selectedEmpty = computed(() => props.selectedPositions.length - selectedWithData.value)

// Message state
const message = ref('')
const showMessage = (text: string) => {
  message.value = text
  setTimeout(() => { message.value = '' }, 3000)
}

const handleApply = () => {
  if (!canApply.value) return
  store.bulkUpdatePoints(props.selectedPositions, {
    hoodAngleDegrees: hoodAngle.value,
    flywheelRPM: flywheelRPM.value
  })
  const created = selectedEmpty.value
  const updated = selectedWithData.value
  const parts: string[] = []
  if (updated > 0) parts.push(`${updated} updated`)
  if (created > 0) parts.push(`${created} created`)
  showMessage(parts.join(', '))
}

// Delete confirmation
const showDeleteModal = ref(false)

const confirmDelete = () => {
  store.bulkRemovePoints(props.selectedPositions)
  showDeleteModal.value = false
  emit('close')
}
</script>

<template>
  <div class="bulk-edit-panel">
    <div class="panel-header">
      <h3>Bulk Edit &mdash; {{ selectedPositions.length }} cells</h3>
      <button class="close-btn" @click="emit('close')">&times;</button>
    </div>

    <div class="info-row">
      <span class="info-item">
        <span class="label">With data</span>
        <span class="value">{{ selectedWithData }}</span>
      </span>
      <span class="info-item">
        <span class="label">Empty</span>
        <span class="value">{{ selectedEmpty }}</span>
      </span>
    </div>

    <div class="input-group">
      <label for="bulk-hood">Hood Angle (deg)</label>
      <input
        id="bulk-hood"
        type="number"
        v-model.number="hoodAngle"
        :min="store.minHoodAngle"
        :max="store.maxHoodAngle"
        step="0.5"
        :class="{ invalid: !isHoodValid }"
      />
      <span class="bounds">{{ store.minHoodAngle }} - {{ store.maxHoodAngle }}°</span>
    </div>

    <div class="input-group">
      <label for="bulk-rpm">Flywheel RPM</label>
      <input
        id="bulk-rpm"
        type="number"
        v-model.number="flywheelRPM"
        :min="store.minFlywheelRPM"
        :max="store.maxFlywheelRPM"
        step="50"
        :class="{ invalid: !isRPMValid }"
      />
      <span class="bounds">{{ store.minFlywheelRPM }} - {{ store.maxFlywheelRPM }} RPM</span>
    </div>

    <div class="button-row">
      <button
        class="btn btn-primary"
        :class="{ disabled: !canApply }"
        :disabled="!canApply"
        @click="handleApply"
      >
        Apply to {{ selectedPositions.length }} cell{{ selectedPositions.length === 1 ? '' : 's' }}
      </button>
      <button
        v-if="selectedWithData > 0"
        class="btn btn-danger"
        @click="showDeleteModal = true"
      >
        Delete
      </button>
    </div>

    <div v-if="message" class="message success">
      {{ message }}
    </div>

    <ConfirmModal
      v-if="showDeleteModal"
      title="Delete Selected Points"
      :message="`Delete ${selectedWithData} calibration point${selectedWithData === 1 ? '' : 's'}?`"
      confirm-text="Delete"
      :danger="true"
      @confirm="confirmDelete"
      @cancel="showDeleteModal = false"
    />
  </div>
</template>

<style scoped>
.bulk-edit-panel {
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

.info-row {
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
</style>
