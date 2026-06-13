# Clave de State Cache

El state se limpia en OnLoad. A partir de ahi se modifican valores pero nunca se borra, ni en SimulationStop ni en el dispose de mechanism data.

## Ciclo de scan SmartComponent
- ``_logging``
- ``_stepWatch``
- ``_lastTick``
- ``lastTime``

## Funcionamiento de SmartComponent
- ``MechanismData``
- ``station``
- ``simConfig``