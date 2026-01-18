import { computed } from 'vue'
import {
  FIELD_IMAGE,
  PLAYABLE_AREA,
  FIELD_DIMENSIONS,
  CALIBRATION_AREA,
  PIXELS_PER_METER,
} from '../constants/fieldConstants'

export function useFieldCoordinates() {
  // Calculate the calibration grid overlay position as CSS percentages
  const gridOverlayStyle = computed(() => {
    // X coordinates: convert meters to pixels, then to percentage
    const leftPixels = PLAYABLE_AREA.topLeft.x + CALIBRATION_AREA.minX * PIXELS_PER_METER.x
    const rightPixels = PLAYABLE_AREA.topLeft.x + CALIBRATION_AREA.maxX * PIXELS_PER_METER.x
    const widthPixels = rightPixels - leftPixels

    // Y coordinates: field Y is inverted (0 at bottom, but image has 0 at top)
    // y=7.0m is near top of playable area, y=1.0m is near bottom
    const topMetersFromPlayableTop = FIELD_DIMENSIONS.heightMeters - CALIBRATION_AREA.maxY
    const bottomMetersFromPlayableTop = FIELD_DIMENSIONS.heightMeters - CALIBRATION_AREA.minY

    const topPixels = PLAYABLE_AREA.topLeft.y + topMetersFromPlayableTop * PIXELS_PER_METER.y
    const bottomPixels = PLAYABLE_AREA.topLeft.y + bottomMetersFromPlayableTop * PIXELS_PER_METER.y
    const heightPixels = bottomPixels - topPixels

    // Convert to percentages of full image
    const left = (leftPixels / FIELD_IMAGE.width) * 100
    const top = (topPixels / FIELD_IMAGE.height) * 100
    const width = (widthPixels / FIELD_IMAGE.width) * 100
    const height = (heightPixels / FIELD_IMAGE.height) * 100

    return {
      left: `${left.toFixed(2)}%`,
      top: `${top.toFixed(2)}%`,
      width: `${width.toFixed(2)}%`,
      height: `${height.toFixed(2)}%`,
    }
  })

  // Get the field image path
  const fieldImagePath = FIELD_IMAGE.path

  return {
    gridOverlayStyle,
    fieldImagePath,
  }
}
