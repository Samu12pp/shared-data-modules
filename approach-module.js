// ============================================================
// approach-guidance.js
// Módulo de guiado de aproximación para el addon A330
// Reutiliza: airportsDB (JSON generado desde OurAirports)
// ============================================================

const EARTH_RADIUS_M = 6371000;
const NM_TO_M = 1852;

function toRad(deg) { return deg * Math.PI / 180; }
function toDeg(rad) { return rad * 180 / Math.PI; }

// --- Geodesia básica ---------------------------------------

function haversineDistance(lat1, lon1, lat2, lon2) {
  const φ1 = toRad(lat1), φ2 = toRad(lat2);
  const Δφ = toRad(lat2 - lat1);
  const Δλ = toRad(lon2 - lon1);
  const a = Math.sin(Δφ / 2) ** 2 +
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  return 2 * EARTH_RADIUS_M * Math.asin(Math.sqrt(a));
}

function bearing(lat1, lon1, lat2, lon2) {
  const φ1 = toRad(lat1), φ2 = toRad(lat2);
  const Δλ = toRad(lon2 - lon1);
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return (toDeg(Math.atan2(y, x)) + 360) % 360;
}

// Punto destino dado origen, rumbo y distancia (fórmula esférica directa)
function destinationPoint(lat, lon, bearingDeg, distanceM) {
  const δ = distanceM / EARTH_RADIUS_M;
  const θ = toRad(bearingDeg);
  const φ1 = toRad(lat), λ1 = toRad(lon);

  const φ2 = Math.asin(
    Math.sin(φ1) * Math.cos(δ) + Math.cos(φ1) * Math.sin(δ) * Math.cos(θ)
  );
  const λ2 = λ1 + Math.atan2(
    Math.sin(θ) * Math.sin(δ) * Math.cos(φ1),
    Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2)
  );

  return { lat: toDeg(φ2), lon: (toDeg(λ2) + 540) % 360 - 180 };
}

// Distancia perpendicular (offset lateral) respecto al eje de una pista
function crossTrackDistance(lat, lon, runwayLat, runwayLon, runwayHeadingDeg) {
  const R = EARTH_RADIUS_M;
  const distToPoint = haversineDistance(runwayLat, runwayLon, lat, lon);
  const bearingToPoint = bearing(runwayLat, runwayLon, lat, lon);
  const runwayBearingRad = toRad(runwayHeadingDeg);
  const bearingDiff = toRad(bearingToPoint) - runwayBearingRad;

  return Math.asin(Math.sin(distToPoint / R) * Math.sin(bearingDiff)) * R;
}

// Distancia a lo largo del eje (para el perfil de descenso de 3°)
function alongTrackDistance(lat, lon, runwayLat, runwayLon, runwayHeadingDeg) {
  const R = EARTH_RADIUS_M;
  const distToPoint = haversineDistance(runwayLat, runwayLon, lat, lon);
  const xtd = crossTrackDistance(lat, lon, runwayLat, runwayLon, runwayHeadingDeg);
  return Math.acos(Math.cos(distToPoint / R) / Math.cos(xtd / R)) * R;
}

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

// --- Parámetros de aproximación según tamaño de pista --------

function getApproachParams(runway) {
  const lengthFt = runway.length_ft;
  const captureDistanceNM = clamp(lengthFt / 800, 6, 15);
  const armDistanceNM = captureDistanceNM * 1.8;
  return { captureDistanceNM, armDistanceNM };
}

// --- Guiado: vectoreo vs. final -------------------------------

const XTD_ALIGNED_THRESHOLD_M = 500; // por debajo de esto, consideramos "alineado"
const INTERCEPT_DISTANCE_NM = 12;    // punto de intercepción por delante del umbral

function getGuidance(myLat, myLon, myHeading, runway) {
  const xtd = crossTrackDistance(myLat, myLon, runway.threshold_lat, runway.threshold_lon, runway.heading);
  const distToThresholdNM = haversineDistance(myLat, myLon, runway.threshold_lat, runway.threshold_lon) / NM_TO_M;

  if (Math.abs(xtd) > XTD_ALIGNED_THRESHOLD_M) {
    // Fase VECTOR: calcular rumbo hacia el punto de intercepción
    const interceptPoint = destinationPoint(
      runway.threshold_lat, runway.threshold_lon,
      (runway.heading + 180) % 360,
      INTERCEPT_DISTANCE_NM * NM_TO_M
    );
    const hdg = bearing(myLat, myLon, interceptPoint.lat, interceptPoint.lon);
    return {
      phase: "VECTOR",
      heading: Math.round(hdg),
      xtd_m: Math.round(xtd),
      distNM: Math.round(distToThresholdNM * 10) / 10,
      interceptPoint
    };
  } else {
    // Fase FINAL: ya alineado, mantener rumbo de pista
    return {
      phase: "FINAL",
      heading: Math.round(runway.heading),
      xtd_m: Math.round(xtd),
      distNM: Math.round(distToThresholdNM * 10) / 10,
      interceptPoint: null
    };
  }
}

// --- Perfil de descenso (pendiente de 3°) ---------------------

function getTargetAltitude(myLat, myLon, runway, slopeDeg = 3) {
  const alongDist = alongTrackDistance(myLat, myLon, runway.threshold_lat, runway.threshold_lon, runway.heading);
  const heightAboveThreshold = alongDist * Math.tan(toRad(slopeDeg));
  return runway.threshold_elev_ft + heightAboveThreshold * 3.28084; // metros a pies
}

// ============================================================
// EJEMPLO DE USO
// ============================================================

function exampleUsage(airportsDB) {
  const airport = airportsDB["SKBO"];
  const runway = airport.runways[0]; // ej. cabecera 13L

  // Posición simulada del avión (ejemplo: 25nm al sureste, desalineado)
  const myLat = 4.40, myLon = -73.95, myHeading = 300;

  const guidance = getGuidance(myLat, myLon, myHeading, runway);
  const targetAlt = getTargetAltitude(myLat, myLon, runway);

  console.log(guidance);
  console.log("Altitud objetivo (ft):", Math.round(targetAlt));
}

// projectToCanvas(lat, lon) -> {x, y} ya debe existir en tu radar actual,
// centrado en la posición del avión y orientado según tu convención
// (norte arriba, o "avión arriba" tipo radar de caza — ajusta según tu caso).

function drawApproachGuidance(ctx, projectToCanvas, myLat, myLon, guidance, runway) {
  const myPos = projectToCanvas(myLat, myLon);
  const thresholdPos = projectToCanvas(runway.threshold_lat, runway.threshold_lon);

  ctx.save();

  if (guidance.phase === "VECTOR") {
    // Color ámbar: "toca girar"
    ctx.strokeStyle = "#ffb020";
    ctx.fillStyle = "#ffb020";

    const interceptPos = projectToCanvas(guidance.interceptPoint.lat, guidance.interceptPoint.lon);

    // Línea avión -> punto de intercepción
    ctx.beginPath();
    ctx.moveTo(myPos.x, myPos.y);
    ctx.lineTo(interceptPos.x, interceptPos.y);
    ctx.setLineDash([6, 4]); // discontinua: "todavía no estás en el eje"
    ctx.lineWidth = 2;
    ctx.stroke();

    // Línea punto de intercepción -> umbral (eje de pista ya definido)
    ctx.beginPath();
    ctx.moveTo(interceptPos.x, interceptPos.y);
    ctx.lineTo(thresholdPos.x, thresholdPos.y);
    ctx.setLineDash([]); // continua: el eje real
    ctx.stroke();

  } else {
    // Color verde: "alineado, mantén"
    ctx.strokeStyle = "#20d060";
    ctx.fillStyle = "#20d060";

    ctx.beginPath();
    ctx.moveTo(myPos.x, myPos.y);
    ctx.lineTo(thresholdPos.x, thresholdPos.y);
    ctx.setLineDash([]);
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // Marcador del umbral
  ctx.beginPath();
  ctx.arc(thresholdPos.x, thresholdPos.y, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();

  // Texto tipo PFD: rumbo objetivo + fase, en una esquina fija (no en el mundo, en pantalla)
  drawHeadingReadout(ctx, guidance);
}

function drawHeadingReadout(ctx, guidance) {
  const x = 20, y = 30; // esquina superior izquierda, ajustar a tu HUD
  ctx.save();
  ctx.font = "bold 18px monospace";
  ctx.fillStyle = guidance.phase === "VECTOR" ? "#ffb020" : "#20d060";

  const label = guidance.phase === "VECTOR"
    ? `GIRA A RUMBO ${String(guidance.heading).padStart(3, "0")}`
    : `MANTÉN RUMBO ${String(guidance.heading).padStart(3, "0")} — ALINEADO`;

  ctx.fillText(label, x, y);
  ctx.font = "12px monospace";
  ctx.fillText(`${guidance.distNM} nm al umbral`, x, y + 18);
  ctx.restore();
}

// --- Uso dentro de tu ciclo de dibujo existente (200-500ms) ---
//
// function radarDrawCycle() {
//   ctx.clearRect(0, 0, canvas.width, canvas.height);
//   drawOwnAircraft(ctx, ...);
//   drawNearbyAirports(ctx, ...); // capa toggleable, la que dejamos pendiente
//
//   if (selectedRunway) {
//     const guidance = getGuidance(myLat, myLon, myHeading, selectedRunway);
//     drawApproachGuidance(ctx, projectToCanvas, myLat, myLon, guidance, selectedRunway);
//   }
// }

module.exports = {
  haversineDistance, bearing, destinationPoint,
  crossTrackDistance, alongTrackDistance,
  getApproachParams, getGuidance, getTargetAltitude,
  exampleUsage,
  drawApproachGuidance, drawHeadingReadout
};
