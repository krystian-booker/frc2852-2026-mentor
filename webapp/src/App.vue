<script setup lang="ts">
import StatusPanel from './components/StatusPanel.vue'
import PositionPanel from './components/PositionPanel.vue'
import ControlPanel from './components/ControlPanel.vue'
import ValidationPanel from './components/ValidationPanel.vue'
import ActionsPanel from './components/ActionsPanel.vue'
import DataTable from './components/DataTable.vue'
import CalibrationGrid from './components/CalibrationGrid.vue'
</script>

<template>
  <div class="app">
    <header class="header">
      <h1>Turret Calibration</h1>
      <span class="subtitle">FRC 2852 - 2026</span>
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
      <span>•</span>
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
}
</style>
