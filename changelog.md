# Changelog

## [Unreleased] 

### Added
- Implementación da cinemática inversa usando `CalculateInverseKinematics`
- Aplicación de valores articulares con `SetJointValues`
- Introdución do array de configuración (`Cf1`, `Cf4`, `Cf6`, `Cfx`)

### Changed
- Actualización da chamada á IK para incluír `WorkObject` e `Tool` activos
- Mellora no control dos valores articulares antes de aplicar o movemento

### Fixed
- Xestión de resultados nulos cando a IK non pode ser calculada (ex: unidades mecánicas diferentes)
- Prevención de movementos inválidos cando non hai solución válida

### Notes
- A IK pode fallar se o target pertence a outra unidade mecánica
- A configuración é necesaria para resolver a postura do robot
- Validación adicional pendente con probas reais

## [Unreleased] 10-05-2026

### Added

### Changed
- Anulada sincronizacion cartesiana por probemas de funcionamiento. Solo auto.

### Fixed
- Arreglo dispose mechdata.
- Sincronizar cambios nos datos entre sourceController e targetController en sincronismo cartesaiano
- Pequeno desaxuste entre movemento e tool usado (Filtro). Detectar e omitir inconsistencias no frame e no tool durante o calculo.

### Notes
- Inicializacion de rsdatacache. ¿Modificase co primeiro evento de cambio de valor?
- Probas con GetRobtarget(string tool, string wobj). Problemas de resolucion de nomes locales. Imposible determinar scope.

## [Unreleased] 16-05-2026

### Added
- Lista desplegable con controladores source disponibles
- Lista desplegable con controladores target disponibles
- Mensaje de reinicio en caso de cambios

### Changed
- Cambio interfaz. Añadido casilla "offline" en Source Controller

### Fixed
- Incompatible mismo GUID en source y target
- Quitar source de posibles target
- InverseKinematics cambiado a version con configuracion de ejes a causa del mal funcionamiento en caso de cambio de controlador. Posiblemente se liaba con la configuracion de ejes.

### Notes

## [Unreleased] 16-05-2026

### Added
- Visualizacion de tool e workobject en SyncJoint
- Eliminar tools y wobj al acabar la sincronizacion
- Cache de task de movimiento sourcecontroller
- Bool "Unidades mecanicas externas"

### Changed

### Fixed
- Latencia SyncCartesian

### Notes

## [Unreleased] 24-05-2026

### Added
- Ocultacion de parametro "Unidades mecánicas exeternas" con CartesianSync a false
- Maximo de reintentos IK para aviso
- Documentacion externa
- Documentacion interna
- Cambio de *modo sincronizacion* en quente
- Cambio de *modo wobj coordinados* en quente

### Changed
- InverseKinematics Async

### Fixed
- Campos privados de CodeBehind en StateCache
- Optimizacion de refresco de graficos: Eliminado GraphicControl.UpdateAll. Se refresca con la propia simulacion.

### Notes
- IK en pruebas
- Posible eliminacion propieaded CooridnatedWObjs: Uso sobrecarga IK con RsWorkObject

## [Unreleased] 31-05-2026

### Added
- Novo campo privado (controller)_targetController 
- Implementacion novo campo: conexion/desconexion e constructor de MechanismData 

### Changed
- Renombrado campo privado (Irc5Controller)_targetController a _targetRsController 

### Fixed

### Notes
- Cambios readme
	- Robot target no ejecuta
	- Si se conectan 2 sources al mismo target se vuelve loco

## [Unreleased] 02-06-2026

### Added
- Novo campo privado (ControllerSimulationConfiguration) _ControllerSimConfig
- Evento OnLoad
- Suscriptores ProjectOnLoad e ProjectClosed (funcionamento comprobado)

### Changed
- Objecto estacion en StateCache
- Station y SimConfig añadido a StateCache
- Eliminada propiedad CoordinateWobj

### Fixed

### Notes
- Problema StateCache no se comparte cuando se edita desde OnLoad (station e project inda non estan creados)
- Todos los controladores virtuales estan configurados para no arrancar en el inicio de la simulacion, solo se arranca el source si es virtual
- Asi mismo se para el source al finalizar la simulacion

## [1.0.0] 13-06-2026

### Added

### Changed
- Eliminada propiedad CoordinateWobj
- Gif de readme

### Fixed
- AutoStop y AutoStart modificados en BeforeStartSimulation y OnPropertyValueChangue respectivamente
- AutoStop se mantiene persistente en OnSimulationStop

### Notes
- Se guarda un backup de la estacion en la carpeta del proyecto

## [1.1.0] 30-06-2026

### Added
- MIT Licence
- Propiedad overwrite tooldata
- Propiedad overwrite wobjdata
- Propiedad retain station data

### Changed
- Eliminado SafeDispose() 
- AddRapidDataToCache: se quita suscripcion evento OnValueChangued con _overwrite
- InitRsDataCache: se coge todos los tools y wobj que haya en la estacion

### Fixed
- Meter documento StateCache en .gitignore

### Notes
- AutoStart a true en SourceController no compensa. Limitacion autostart. Revisar antes de arrancar simulacion.

---

### TODO
- Documentar: SimConfiguration, AddRapidDataToCache
- Actualizar README
- Guardar targerController y sourceController COMO CONTROLLERINFO cuando se cambia de valor la propiedad
- Excepcion: Perdida de conexion controlador despois de conectar
- Marcar un rscontroller como "usado"
- Marcar un rscontroller como "dummy"
- Metricas de Calidad de conexion
- Opcion sin datos locales. Sin datos locales se puede reducir la latencia de CartesianSync