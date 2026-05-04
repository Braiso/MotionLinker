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