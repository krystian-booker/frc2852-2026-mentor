<script setup lang="ts">
import { computed } from 'vue'
import { useNTBoolean, useNTDouble, useNTInteger } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { TOPICS } from '../constants/topics'

const store = useCalibrationStore()

// Subscribe to robot state needed for validation
const hoodAtPosition = useNTBoolean(TOPICS.HOOD_AT_POSITION, false)
const flywheelAtSetpoint = useNTBoolean(TOPICS.FLYWHEEL_AT_SETPOINT, false)
const gridRow = useNTInteger(TOPICS.GRID.CURRENT_ROW, 0)
const gridCol = useNTInteger(TOPICS.GRID.CURRENT_COL, 0)
const inputHoodAngle = useNTDouble(TOPICS.INPUT_HOOD_ANGLE, 25)
const inputFlywheelRPM = useNTDouble(TOPICS.INPUT_FLYWHEEL_RPM, 3000)

// Compute validation state locally
const hoodAngleValid = computed(() => store.isHoodAngleValid(inputHoodAngle.value))
const flywheelRPMValid = computed(() => store.isFlywheelRPMValid(inputFlywheelRPM.value))
const duplicatePointWarning = computed(() => store.hasPointAt(gridRow.value, gridCol.value))

// Compute overall readyToSave state
const readyToSave = computed(() => {
  return hoodAtPosition.value &&
         flywheelAtSetpoint.value &&
         hoodAngleValid.value &&
         flywheelRPMValid.value
})

// Build validation warning message
const validationWarning = computed(() => {
  const warnings: string[] = []

  if (!hoodAtPosition.value) {
    warnings.push('Hood not at setpoint.')
  }
  if (!flywheelAtSetpoint.value) {
    warnings.push('Flywheel not at setpoint.')
  }
  if (!hoodAngleValid.value) {
    warnings.push(`Hood angle ${inputHoodAngle.value.toFixed(1)}° outside valid range (${store.minHoodAngle}-${store.maxHoodAngle}°).`)
  }
  if (!flywheelRPMValid.value) {
    warnings.push(`Flywheel RPM ${inputFlywheelRPM.value.toFixed(0)} outside valid range (${store.minFlywheelRPM}-${store.maxFlywheelRPM}).`)
  }

  return warnings.join(' ')
})
</script>

<template>
  <div class="validation-panel">
    <h3>Validation</h3>

    <div class="ready-status" :class="{ ready: readyToSave }">
      <span class="icon">{{ readyToSave ? '✓' : '○' }}</span>
      {{ readyToSave ? 'Ready to Save' : 'Not Ready' }}
    </div>

    <div class="validation-checks">
      <div class="check-item" :class="{ valid: hoodAtPosition }">
        <span class="icon">{{ hoodAtPosition ? '✓' : '✗' }}</span>
        Hood At Position
      </div>
      <div class="check-item" :class="{ valid: flywheelAtSetpoint }">
        <span class="icon">{{ flywheelAtSetpoint ? '✓' : '✗' }}</span>
        Flywheel At Setpoint
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

    <div class="warning-badge" :class="{ hidden: !duplicatePointWarning }">
      Point already exists at this grid cell - will update existing
    </div>

    <div class="warning-message" :class="{ hidden: !validationWarning }">
      {{ validationWarning || '&nbsp;' }}
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

.warning-badge.hidden {
  visibility: hidden;
}

.warning-message {
  padding: 8px;
  border-radius: 4px;
  background: #1a1a2e;
  color: #aaa;
  font-size: 12px;
  line-height: 1.4;
}

.warning-message.hidden {
  visibility: hidden;
}
</style>
