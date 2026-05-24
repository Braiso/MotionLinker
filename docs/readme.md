# Readme

## Descripcion
MotionLinker es un Smart Component para RobotStudio que sincroniza el movimiento de un controlador ABB origen con un controlador virtual destino. Soporta sincronización por ejes y sincronización cartesiana.

## Funcionalidades
- Sincronización de movimiento por ejes (Joint)
- Sincronización de movimiento cartesiano
- Modo manual y automático
- Búsqueda automática de controladores
- Soporte para controladores online/offline
- Sincronización de tooldata y wobjdata globales y locales entre Source y Target
- Actualización y visualizacion automática de tooldata y wobjdata activos en Target
- Soporte para workobjects coordinados

## Requisitos
- RobotStudio 2025
- RobotStudio SDK 2025
- PC SDK 2025
- Framework .NET 4.8
- Estación con la misma unidad mecánica y controlador *dummy*
- La tarea ``T_Rob1`` debe tener un programa principal que no se detenga pero que tampoco ejecute instrucciones de movimiento que interfieran con la sincronización

# Configuración/Propiedades
| Propiedad        |   Tipo | Descripción               |
| ---------------- | -----: | ------------------------- |
| SourceController | String | Controlador origen        |
| TargetController | String | Controlador destino       |
| OnlineSource     |   Bool | Fuente online             |
| Cartesian        |   Bool | Sincronización cartesiana |
| CoordinatedWObjs |   Bool | WObj coordinado           |


## Uso rápido

Ejemplo:

1. Añadir MotionLinker a la estación.
2. Seleccionar SourceController.
3. Seleccionar TargetController.
4. Elegir modo Joint o Cartesian.
5. Ejecutar simulación.

![Descripción del gif](./assets/MotionLinker.gif)

## Limitaciones conocidas
- La sincronización cartesiana puede no ser fiable en modo Manual, por lo que se cambia automáticamente a sincronización Joint en ese modo.
- Solo se usa T_ROB1.
- RobotStudioSDK y PCSDK solo es compatible con Framwork.NET 4.8 

## Documentación
[Arquitectura y flujo de funcionamiento](arquitectura.md)

