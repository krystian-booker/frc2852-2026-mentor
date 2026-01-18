import { ref, onMounted, type Ref } from 'vue'
import { NetworkTables, NetworkTablesTypeInfos, type NetworkTablesTopic } from 'ntcore-ts-client'

const ntInstance = NetworkTables.getInstanceByURI('localhost')

// Cache for published topics to avoid re-publishing
const doubleTopics = new Map<string, NetworkTablesTopic<number>>()
const booleanTopics = new Map<string, NetworkTablesTopic<boolean>>()

export function useNTConnection() {
  const connected = ref(false)
  const serverAddress = ref('localhost')

  const connect = (address: string) => {
    serverAddress.value = address
    // ntcore-ts-client auto-connects, we just track the state
  }

  onMounted(() => {
    ntInstance.addRobotConnectionListener((isConnected) => {
      connected.value = isConnected
    }, true)
  })

  return { connected, serverAddress, connect }
}

export function useNTBoolean(topic: string, defaultValue: boolean = false): Ref<boolean> {
  const value = ref(defaultValue)

  onMounted(() => {
    const ntTopic = ntInstance.createTopic<boolean>(topic, NetworkTablesTypeInfos.kBoolean)
    ntTopic.subscribe((newValue) => {
      if (newValue !== null) {
        value.value = newValue
      }
    })
  })

  return value
}

export function useNTDouble(topic: string, defaultValue: number = 0): Ref<number> {
  const value = ref(defaultValue)

  onMounted(() => {
    const ntTopic = ntInstance.createTopic<number>(topic, NetworkTablesTypeInfos.kDouble)
    ntTopic.subscribe((newValue) => {
      if (newValue !== null) {
        value.value = newValue
      }
    })
  })

  return value
}

export function useNTInteger(topic: string, defaultValue: number = 0): Ref<number> {
  const value = ref(defaultValue)

  onMounted(() => {
    const ntTopic = ntInstance.createTopic<number>(topic, NetworkTablesTypeInfos.kInteger)
    ntTopic.subscribe((newValue) => {
      if (newValue !== null) {
        value.value = newValue
      }
    })
  })

  return value
}

export function useNTString(topic: string, defaultValue: string = ''): Ref<string> {
  const value = ref(defaultValue)

  onMounted(() => {
    const ntTopic = ntInstance.createTopic<string>(topic, NetworkTablesTypeInfos.kString)
    ntTopic.subscribe((newValue) => {
      if (newValue !== null) {
        value.value = newValue
      }
    })
  })

  return value
}

async function getOrCreateDoubleTopic(topic: string): Promise<NetworkTablesTopic<number>> {
  if (!doubleTopics.has(topic)) {
    const ntTopic = ntInstance.createTopic<number>(topic, NetworkTablesTypeInfos.kDouble)
    await ntTopic.publish()
    doubleTopics.set(topic, ntTopic)
  }
  return doubleTopics.get(topic)!
}

async function getOrCreateBooleanTopic(topic: string): Promise<NetworkTablesTopic<boolean>> {
  if (!booleanTopics.has(topic)) {
    const ntTopic = ntInstance.createTopic<boolean>(topic, NetworkTablesTypeInfos.kBoolean)
    await ntTopic.publish()
    booleanTopics.set(topic, ntTopic)
  }
  return booleanTopics.get(topic)!
}

export async function publishDouble(topic: string, value: number): Promise<void> {
  const ntTopic = await getOrCreateDoubleTopic(topic)
  ntTopic.setValue(value)
}

export async function publishBoolean(topic: string, value: boolean): Promise<void> {
  const ntTopic = await getOrCreateBooleanTopic(topic)
  ntTopic.setValue(value)
}

export async function triggerBoolean(topic: string): Promise<void> {
  const ntTopic = await getOrCreateBooleanTopic(topic)
  ntTopic.setValue(true)
  // Reset after a short delay for edge-triggered topics
  setTimeout(() => {
    ntTopic.setValue(false)
  }, 100)
}

export { ntInstance }
