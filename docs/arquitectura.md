# Módulos principales

## CodeBehind.cs

Responsable del ciclo de vida del Smart Component:

- Inicio y parada de simulación
- Validación de propiedades
- Descubrimiento de controladores
- Gestión de eventos
- Ejecución de sincronización

Esto se ve en OnSimulationStart, OnSimulationStop y OnSimulationStep.

## MechanismData.cs

Contiene la lógica principal:

- Gestión de controladores
- Cache de tooldata y wobjdata
- Conversión RAPID → RobotStudio
- Sincronización Joint y Cartesian
- Gestión de recursos

## ControllerHelper.cs

Funciones auxiliares:

- Búsqueda de controladores
- Conexión a controladores
- Logging de eventos

# Flujo de funcinamiento
````mermaid
flowchart TD
    A[Inicio simulación] --> B[Leer propiedades]
    B --> C[Buscar controladores]
    C --> D[Crear MechanismData]
    D --> E[Inicializar cache]
    E --> F[Iniciar simulación]
    F --> G[Sincronizar movimiento]
    G --> H[Cerrar recursos]
````