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

### TODO
- Sincronizar cambios nos datos entre sourceController e targetController en sincronismo cartesaiano
- Ver que pasa con ExternalAxis. En principio esta implementado pero os valores no son coeherentes
- Cambio de modo en quente
- Arreglo dispose mechdata (feo).
- Pequeno desaxuste entre movemento e tool usado. Filtro.
- Inicializacion de rsdatacache. ¿Modificase co primeiro evento de cambio de valor?

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

### TODO
- Lista desplegable con controladores activos
- Ver que pasa con ExternalAxis. En principio esta implementado pero os valores no son coeherentes
- Cambio de modo en quente
- Calidad de conexion
- Optimizacion de refresco de graficos
	- DetailLevel	