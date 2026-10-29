// ============================================================
// 767-300ER-loader.js
// Carga el Boeing 767-300ER de esta carpeta en GeoFS como avion "local"
// (sin pasar por el servidor de GeoFS). Pegar en la consola del navegador
// (F12) con GeoFS abierto, o usar como userscript / bookmarklet.
//
// GeoFS lee <BASE>aircraft.json y, al entrar en la vista de cabina,
// <BASE>cockpit/cockpit.json. Los modelos .glb se piden a la misma ruta.
// ============================================================
(function () {
  // Rama donde estan los archivos. Cuando se fusione en main, cambiar a 'main'.
  var BRANCH = 'claude/laughing-tesla-24hh6c';
  var BASE = 'https://raw.githubusercontent.com/Samu12pp/shared-data-modules/refs/heads/' + BRANCH + '/aircraft/767-300ER/';
  var ID = 'b763er-local';

  if (!window.geofs || !geofs.aircraft || !geofs.aircraft.instance) {
    alert('Abre GeoFS y espera a que cargue antes de ejecutar este script.');
    return;
  }
  geofs.aircraftList = geofs.aircraftList || {};
  geofs.aircraftList[ID] = { id: ID, name: 'Boeing 767-300ER', path: BASE, local: true };
  geofs.aircraft.instance.change(ID);
})();
