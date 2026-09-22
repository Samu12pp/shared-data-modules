// ============================================================
// efis-panel-module.js
// Módulo del panel EFIS (control panel, lado CAPT/FO) para el addon A330
// Reutiliza: airportsDB (JSON por ICAO) y approach-module.js (geodesia/guiado)
// Cada función es pura: el estado (activo/inactivo, modo actual, rango
// actual) lo mantiene el addon; este módulo solo calcula.
// ============================================================

const { haversineDistance, bearing, getGuidance, getTargetAltitude } = require('./approach-module.js');

const NM_TO_M = 1852;

function angularDiff(a, b) {
  const diff = Math.abs(a - b) % 360;
  return diff > 180 ? 360 - diff : diff;
}

// --- ARPT: aeropuertos cercanos --------------------------------

function findNearbyAirports(myLat, myLon, airportsDB, rangeNM) {
  const rangeM = rangeNM * NM_TO_M;
  const results = [];

  for (const icao in airportsDB) {
    const airport = airportsDB[icao];
    const distM = haversineDistance(myLat, myLon, airport.lat, airport.lon);
    if (distM <= rangeM) {
      results.push({
        icao: airport.icao,
        lat: airport.lat,
        lon: airport.lon,
        distanceNM: distM / NM_TO_M,
        bearing: bearing(myLat, myLon, airport.lat, airport.lon)
      });
    }
  }

  return results.sort((a, b) => a.distanceNM - b.distanceNM);
}

function drawNearbyAirports(ctx, projectToCanvas, airports) {
  ctx.save();
  ctx.fillStyle = '#20d0ff';
  ctx.font = '10px monospace';

  for (const apt of airports) {
    const pos = projectToCanvas(apt.lat, apt.lon);
    ctx.beginPath();
    ctx.moveTo(pos.x, pos.y - 4);
    ctx.lineTo(pos.x + 4, pos.y + 3);
    ctx.lineTo(pos.x - 4, pos.y + 3);
    ctx.closePath();
    ctx.fill();
    ctx.fillText(apt.icao, pos.x + 6, pos.y + 3);
  }

  ctx.restore();
}

// --- WPT (repurposado): guiado de aproximación en directo ------
//
// En vez de listar waypoints fuera del plan, WPT activa/desactiva la capa
// de guiado de aproximación (getGuidance/getTargetAltitude/drawApproachGuidance
// de approach-module.js) hacia el aeropuerto y pista de destino.

function resolveApproachRunway(airport, selectedRunwayId, myLat, myLon, myHeading) {
  if (!airport || !airport.runways || airport.runways.length === 0) {
    return { runway: null, autoSelected: false };
  }

  if (selectedRunwayId) {
    const chosen = airport.runways.find(rw => rw.id === selectedRunwayId);
    if (chosen) return { runway: chosen, autoSelected: false };
  }

  // Sin pista elegida en el FMS: se aproxima por la cabecera cuyo rumbo
  // mejor encaja con la posición y el rumbo actuales del avión.
  let best = null;
  let bestScore = Infinity;
  for (const rw of airport.runways) {
    const bearingToThreshold = bearing(myLat, myLon, rw.threshold_lat, rw.threshold_lon);
    const score = angularDiff(bearingToThreshold, rw.heading) + angularDiff(myHeading, rw.heading);
    if (score < bestScore) {
      bestScore = score;
      best = rw;
    }
  }

  return { runway: best, autoSelected: true };
}

function createAutoRunwayWarning(now = Date.now()) {
  return {
    message: 'PISTA NO SELECCIONADA — AUTO',
    color: '#ff2020',
    createdAt: now,
    durationMs: 10000
  };
}

function isWarningActive(warning, now = Date.now()) {
  return !!warning && (now - warning.createdAt) < warning.durationMs;
}

function drawAutoRunwayWarning(ctx, warning, now = Date.now()) {
  if (!isWarningActive(warning, now)) return;

  ctx.save();
  ctx.font = 'bold 16px monospace';
  ctx.fillStyle = warning.color;
  ctx.textAlign = 'center';
  ctx.fillText(warning.message, ctx.canvas.width / 2, 40);
  ctx.restore();
}

// Wiring de referencia (el addon decide cuándo llamarlo):
//
// function onWptPressed() {
//   const { runway, autoSelected } = resolveApproachRunway(
//     destinationAirport, fmsSelectedRunwayId, myLat, myLon, myHeading
//   );
//   if (autoSelected) activeWarning = createAutoRunwayWarning();
//   approachRunway = runway; // null si el aeropuerto no tiene pistas
// }
//
// function radarDrawCycle() {
//   if (approachRunway) {
//     const guidance = getGuidance(myLat, myLon, myHeading, approachRunway);
//     drawApproachGuidance(ctx, projectToCanvas, myLat, myLon, guidance, approachRunway);
//   }
//   drawAutoRunwayWarning(ctx, activeWarning);
// }

// --- RADAR MODE: toggle WX <-> Terreno, independiente por lado --

const RADAR_MODES = ['WX', 'TERR'];

function toggleRadarMode(currentMode) {
  const idx = RADAR_MODES.indexOf(currentMode);
  return RADAR_MODES[(idx + 1) % RADAR_MODES.length];
}

// --- ZOOM IN / ZOOM OUT: rango del ND ---------------------------

const ND_RANGES_NM = [10, 20, 40, 80, 160, 320];

function zoomIn(currentRangeNM) {
  const idx = Math.max(0, ND_RANGES_NM.indexOf(currentRangeNM));
  return ND_RANGES_NM[Math.max(0, idx - 1)];
}

function zoomOut(currentRangeNM) {
  const idx = Math.max(0, ND_RANGES_NM.indexOf(currentRangeNM));
  return ND_RANGES_NM[Math.min(ND_RANGES_NM.length - 1, idx + 1)];
}

module.exports = {
  findNearbyAirports,
  drawNearbyAirports,
  resolveApproachRunway,
  createAutoRunwayWarning,
  isWarningActive,
  drawAutoRunwayWarning,
  RADAR_MODES,
  toggleRadarMode,
  ND_RANGES_NM,
  zoomIn,
  zoomOut
};
