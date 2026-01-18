<script setup lang="ts">
import { ref, watch, onMounted } from 'vue'
import { useNTDouble, useNTBoolean, publishDouble } from '../composables/useNetworkTables'

// Read actual values from robot
const actualHoodAngle = useNTDouble('/TurretCalibration/Actual/HoodAngle', 25)
const actualFlywheelRPM = useNTDouble('/TurretCalibration/Actual/FlywheelRPM', 3000)
const hoodAtPosition = useNTBoolean('/TurretCalibration/Actual/HoodAtPosition', false)
const flywheelAtSetpoint = useNTBoolean('/TurretCalibration/Actual/FlywheelAtSetpoint', false)

// Input values (local state, published to NT)
const inputHoodAngle = ref(25)
const inputFlywheelRPM = ref(3000)

// Min/max values (match Java constants)
const hoodMin = 15
const hoodMax = 60
const flywheelMin = 1000
const flywheelMax = 6000

// Publish changes to NetworkTables
watch(inputHoodAngle, (value) => {
  publishDouble('/TurretCalibration/Input/HoodAngle', value)
})

watch(inputFlywheelRPM, (value) => {
  publishDouble('/TurretCalibration/Input/FlywheelRPM', value)
})

// Initialize with NT values on mount
onMounted(() => {
  // Optionally read initial values from NT
  publishDouble('/TurretCalibration/Input/HoodAngle', inputHoodAngle.value)
  publishDouble('/TurretCalibration/Input/FlywheelRPM', inputFlywheelRPM.value)
})
</script>

<template>
  <div class="control-panel">
    <h3>Controls</h3>

    <div class="control-group">
      <div class="control-header">
        <label>Hood Angle</label>
        <span class="setpoint-indicator" :class="{ reached: hoodAtPosition }">
          {{ hoodAtPosition ? 'At Position' : 'Moving...' }}
        </span>
      </div>

      <div class="slider-row">
        <input
          type="range"
          v-model.number="inputHoodAngle"
          :min="hoodMin"
          :max="hoodMax"
          step="0.5"
        />
        <input
          type="number"
          v-model.number="inputHoodAngle"
          :min="hoodMin"
          :max="hoodMax"
          step="0.5"
          class="value-input"
        />
        <span class="unit">deg</span>
      </div>

      <div class="actual-value">
        Actual: {{ actualHoodAngle.toFixed(1) }}°
      </div>
    </div>

    <div class="control-group">
      <div class="control-header">
        <label>Flywheel RPM</label>
        <span class="setpoint-indicator" :class="{ reached: flywheelAtSetpoint }">
          {{ flywheelAtSetpoint ? 'At Setpoint' : 'Spinning Up...' }}
        </span>
      </div>

      <div class="slider-row">
        <input
          type="range"
          v-model.number="inputFlywheelRPM"
          :min="flywheelMin"
          :max="flywheelMax"
          step="50"
        />
        <input
          type="number"
          v-model.number="inputFlywheelRPM"
          :min="flywheelMin"
          :max="flywheelMax"
          step="50"
          class="value-input"
        />
        <span class="unit">RPM</span>
      </div>

      <div class="actual-value">
        Actual: {{ actualFlywheelRPM.toFixed(0) }} RPM
      </div>
    </div>
  </div>
</template>

<style scoped>
.control-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
}

h3 {
  margin: 0 0 16px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.control-group {
  margin-bottom: 20px;
}

.control-group:last-child {
  margin-bottom: 0;
}

.control-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.control-header label {
  font-weight: 500;
  font-size: 14px;
}

.setpoint-indicator {
  font-size: 12px;
  padding: 2px 8px;
  border-radius: 10px;
  background: #444;
  color: #aaa;
}

.setpoint-indicator.reached {
  background: rgba(76, 175, 80, 0.2);
  color: #4caf50;
}

.slider-row {
  display: flex;
  align-items: center;
  gap: 12px;
}

input[type="range"] {
  flex: 1;
  height: 6px;
  border-radius: 3px;
  background: #1a1a2e;
  appearance: none;
  outline: none;
}

input[type="range"]::-webkit-slider-thumb {
  appearance: none;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #3f51b5;
  cursor: pointer;
}

input[type="range"]::-webkit-slider-thumb:hover {
  background: #5c6bc0;
}

.value-input {
  width: 70px;
  padding: 6px 8px;
  border: 1px solid #444;
  border-radius: 4px;
  background: #1a1a2e;
  color: #eee;
  font-size: 14px;
  text-align: right;
}

.value-input:focus {
  outline: none;
  border-color: #3f51b5;
}

.unit {
  font-size: 12px;
  color: #888;
  width: 30px;
}

.actual-value {
  margin-top: 6px;
  font-size: 12px;
  color: #888;
  font-family: 'Consolas', monospace;
}
</style>
