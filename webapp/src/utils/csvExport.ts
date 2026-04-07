/**
 * Exports CSV content using File System Access API (if available) or fallback download.
 */
export async function exportCSV(content: string, suggestedFilename: string): Promise<boolean> {
  // Try File System Access API first (for direct save to source tree)
  if ('showSaveFilePicker' in window) {
    try {
      const handle = await (window as any).showSaveFilePicker({
        suggestedName: suggestedFilename,
        types: [
          {
            description: 'CSV Files',
            accept: { 'text/csv': ['.csv'] }
          }
        ]
      })

      const writable = await handle.createWritable()
      await writable.write(content)
      await writable.close()

      return true
    } catch (e: any) {
      // User cancelled or error
      if (e.name !== 'AbortError') {
        console.error('Failed to save file:', e)
      }
      return false
    }
  }

  // Fallback: trigger download
  const blob = new Blob([content], { type: 'text/csv;charset=utf-8;' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.download = suggestedFilename
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
  URL.revokeObjectURL(url)

  return true
}

/**
 * Checks if File System Access API is available.
 */
export function hasFileSystemAccess(): boolean {
  return 'showSaveFilePicker' in window
}
