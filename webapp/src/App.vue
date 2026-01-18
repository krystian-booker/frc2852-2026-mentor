<script setup lang="ts">
import { onMounted, onUnmounted, watch } from 'vue'
import { useNTDouble, useNTInteger, useNTBoolean, useNTStringArray, triggerBoolean } from './composables/useNetworkTables'
import { useCalibrationStore } from './stores/calibration'
import { exportCSV } from './utils/csvExport'
import StatusPanel from './components/StatusPanel.vue'
import PositionPanel from './components/PositionPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import ValidationPanel from './components/ValidationPanel.vue'
import ActionsPanel from './components/ActionsPanel.vue'
import DataTable from './components/DataTable.vue'
import CalibrationGrid from './components/CalibrationGrid.vue'

const store = useCalibrationStore()

// Subscribe to values needed for keyboard shortcut save
const readyToSave = useNTBoolean('/TurretCalibration/Validation/ReadyToSave', false)

// Subscribe to robot data for synchronization (robot is source of truth)
const robotDataVersion = useNTInteger('/TurretCalibration/Data/Version', 0)
const robotDataPoints = useNTStringArray('/TurretCalibration/Data/Points', [])

// Subscribe to robot constants for validation bounds
const minHoodAngle = useNTDouble('/TurretCalibration/Constants/MinHoodAngle', 0)
const maxHoodAngle = useNTDouble('/TurretCalibration/Constants/MaxHoodAngle', 45)
const minFlywheelRPM = useNTDouble('/TurretCalibration/Constants/MinFlywheelRPM', 1000)
const maxFlywheelRPM = useNTDouble('/TurretCalibration/Constants/MaxFlywheelRPM', 6000)

// Watch for data version changes and sync from robot
watch(robotDataVersion, (newVersion, oldVersion) => {
  if (newVersion > 0 && newVersion !== oldVersion) {
    const synced = store.syncFromRobot(robotDataPoints.value)
    console.log(`Synced ${synced} calibration points from robot (version ${newVersion})`)
  }
})

// Watch for constants changes and update store validation bounds
watch(
  [minHoodAngle, maxHoodAngle, minFlywheelRPM, maxFlywheelRPM],
  ([minHood, maxHood, minRPM, maxRPM]) => {
    store.setValidationBounds(minHood, maxHood, minRPM, maxRPM)
  },
  { immediate: true }
)

const handleKeydown = async (event: KeyboardEvent) => {
  // Check for Ctrl (or Cmd on Mac) key
  const isModified = event.ctrlKey || event.metaKey

  if (isModified) {
    switch (event.key.toLowerCase()) {
      case 's':
        // Ctrl+S = Save Point (robot-side only - data syncs back via NT)
        event.preventDefault()
        if (readyToSave.value) {
          await triggerBoolean('/TurretCalibration/SavePoint')
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
        <CalibrationGrid />
      </div>

      <div class="right-column">
        <DataTable />
      </div>
    </main>

    <footer class="footer">
      <span>Connect to robot at port 5810 (NetworkTables)</span>
      <span>|</span>
      <span>Export saves to src/main/deploy/calibration/</span>
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
