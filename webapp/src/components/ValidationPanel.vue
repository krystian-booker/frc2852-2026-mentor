<script setup lang="ts">
import { useNTBoolean, useNTString } from '../composables/useNetworkTables'

const readyToSave = useNTBoolean('/TurretCalibration/Validation/ReadyToSave', false)
const validationWarning = useNTString('/TurretCalibration/Validation/Warning', '')
const distanceValid = useNTBoolean('/TurretCalibration/Validation/DistanceValid', true)
const hoodAngleValid = useNTBoolean('/TurretCalibration/Validation/HoodAngleValid', true)
const flywheelRPMValid = useNTBoolean('/TurretCalibration/Validation/FlywheelRPMValid', true)
const duplicatePointWarning = useNTBoolean('/TurretCalibration/Validation/DuplicatePoint', false)
</script>

<template>
  <div class="validation-panel">
    <h3>Validation</h3>

    <div class="ready-status" :class="{ ready: readyToSave }">
      <span class="icon">{{ readyToSave ? '✓' : '○' }}</span>
      {{ readyToSave ? 'Ready to Save' : 'Not Ready' }}
    </div>

    <div class="validation-checks">
      <div class="check-item" :class="{ valid: distanceValid }">
        <span class="icon">{{ distanceValid ? '✓' : '✗' }}</span>
        Distance Valid
      </div>
      <div class="check-item" :class="{ valid: hoodAngleValid }">
        <span class="icon">{{ hoodAngleValid ? '✓' : '✗' }}</span>
        Hood Angle Valid
      </div>
      <div class="check-item" :class="{ valid: flywheelRPMValid }">
        <span class="icon">{{ flywheelRPMValid ? '✓' : '✗' }}</span>
        Flywheel RPM Valid
      </div>
    </div>

    <div v-if="duplicatePointWarning" class="warning-badge">
      Point already exists at this grid cell - will update existing
    </div>

    <div v-if="validationWarning" class="warning-message">
      {{ validationWarning }}
    </div>
  </div>
</template>

<style scoped>
.validation-panel {
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

.ready-status {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 12px;
  border-radius: 6px;
  background: rgba(244, 67, 54, 0.1);
  border: 1px solid rgba(244, 67, 54, 0.3);
  color: #f44336;
  font-weight: 500;
  margin-bottom: 12px;
}

.ready-status.ready {
  background: rgba(76, 175, 80, 0.1);
  border-color: rgba(76, 175, 80, 0.3);
  color: #4caf50;
}

.ready-status .icon {
  font-size: 18px;
}

.validation-checks {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-bottom: 12px;
}

.check-item {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 6px 10px;
  border-radius: 4px;
  background: #1a1a2e;
  font-size: 12px;
  color: #f44336;
}

.check-item.valid {
  color: #4caf50;
}

.check-item .icon {
  font-size: 12px;
}

.warning-badge {
  padding: 8px 12px;
  border-radius: 4px;
  background: rgba(255, 152, 0, 0.1);
  border: 1px solid rgba(255, 152, 0, 0.3);
  color: #ff9800;
  font-size: 12px;
  margin-bottom: 8px;
}

.warning-message {
  padding: 8px;
  border-radius: 4px;
  background: #1a1a2e;
  color: #aaa;
  font-size: 12px;
  line-height: 1.4;
}
</style>
