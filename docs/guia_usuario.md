# DOMMUS — Guía de uso

Sistema de control del hogar: alarma, calefacción, agua y sensores.

---

## La pantalla

La pantalla es redonda y táctil. Se navega deslizando el dedo o girando el mando lateral.

Hay 5 pantallas principales que se alcanzan deslizando:

```
        [ ALARMA ]
            ↑
[ HISTORIAL ] ← [ INICIO ] → [ RELÉS ]
            ↓
        [ AJUSTES ]
```

- **INICIO** — pantalla principal. Muestra temperatura, presencia, estado de la alarma y del agua.
- **RELÉS** — control manual de la electroválvula de agua, la sirena y la calefacción.
- **ALARMA** — armar y desarmar el sistema de intrusión.
- **HISTORIAL** — gráficas de temperatura, mapa de presencia y registro de eventos.
- **AJUSTES** — brillo, horario de atenuación, cambio de PIN.

---

## Alarma de intrusión

### Armar la alarma
1. Desliza a la pantalla **ALARMA**
2. Pulsa **ARMAR**
3. Introduce el PIN de 4 dígitos
4. Tienes **2 minutos** para salir de casa — una cuenta atrás lo indica
5. Al llegar a cero la alarma queda activa

### Desarmar la alarma
1. Al entrar en casa el sistema detecta movimiento y emite pitidos durante **30 segundos**
2. En ese tiempo, ve a la pantalla **ALARMA** y pulsa **DESARMAR** (o pulsa la notificación del móvil)
3. Introduce el PIN
4. Si no introduces el PIN a tiempo, salta la alarma completa (sirena + notificación)

### Si salta la alarma
La pantalla muestra una alerta roja con el motivo. Para silenciarla:
1. Pulsa **INTRODUCIR PIN** en la pantalla de alerta
2. Introduce el PIN
3. La sirena se apaga y la alarma queda desarmada

---

## Alarmas de inundación y humo

Estas alarmas son **automáticas** — saltan solas sin necesidad de tener la alarma armada.

### Inundación
- El sensor de agua detecta humedad en el suelo
- **Las válvulas de agua se cierran automáticamente**
- Llega una notificación al móvil
- La pantalla muestra alerta roja: **INUNDACION**
- Para desactivar la alerta: pulsa **DESACTIVAR** e introduce el PIN
- Una vez solucionado el problema, abre el agua manualmente desde la pantalla **RELÉS**

### Humo / calor (cocina)
- El detector de la cocina se activa
- **La sirena suena automáticamente**
- Llega una notificación al móvil
- La pantalla muestra alerta roja: **HUMO**
- Para desactivar: pulsa **DESACTIVAR** e introduce el PIN

---

## Agua

En la pantalla **RELÉS**, el panel verde grande en la parte inferior es la **electroválvula de agua**.

- **Verde oscuro** → agua abierta (normal)
- **Verde vivo** → agua abierta y relé activo
- **Pulsar** → abre o cierra la válvula

> La válvula se cierra automáticamente si se detecta inundación.
> Para reabrirla hay que hacerlo manualmente desde esta pantalla.

---

## Calefacción

En la pantalla **RELÉS**, la zona central tiene tres botones:

| Botón | Qué hace |
|-------|----------|
| **MANUAL** | Enciende la calefacción sin condiciones. La apaga pulsando de nuevo. |
| **CONSIGNA** | Mantiene la temperatura en el valor marcado. Sube/baja con el botón numérico a la derecha. |
| **PROGRAMA** | Sigue el horario semanal configurado (bloques de 4 horas, CONFORT o ECO). |

El botón activo se ilumina en azul.

---

## Corte de luz

Si hay un corte de suministro eléctrico en casa llega una notificación al móvil:
**"Corte de luz"**

Cuando se restaura llega otra notificación de confirmación.
No hace falta hacer nada — el sistema arranca solo cuando vuelve la luz.

---

## Notificaciones en el móvil

La app **Home Assistant** en el móvil recibe alertas automáticas:

| Notificación | Qué significa |
|---|---|
| 🚨 INUNDACIÓN DETECTADA | Sensor de agua activo. Válvulas ya cerradas. |
| 🔥 ALARMA HUMO/CALOR | Detector cocina activo. Sirena encendida. |
| 🚨 POSIBLE INTRUSIÓN | Movimiento con alarma armada. Tienes 30s para el PIN. |
| 🔴 ALARMA ACTIVADA | La alarma está sonando. |
| ⚡ CORTE DE LUZ | Sin alimentación eléctrica. |
| ✅ Luz restaurada | La corriente ha vuelto. |
| 🛡️ Alarma armada | Alguien armó el sistema. |
| 🔓 Alarma desarmada | Alguien desarmó el sistema. |

---

## La pantalla se apaga sola

Normal — se atenúa después de unos minutos sin uso para ahorrar energía.
Un toque en cualquier punto la reactiva.

---

## Qué NO tocar

- **Ajustes → Cambiar PIN** — solo si tienes el PIN actual y quieres cambiarlo
- **Sirena** (panel rojo en la pantalla RELÉS) — pide confirmación antes de activarse, pero úsala solo en caso real de necesidad

---

## Resumen rápido de emergencias

| Situación | Qué hacer |
|---|---|
| Inundación | Nada — las válvulas se cierran solas. Llama al propietario. |
| Humo / fuego | Evacua. La sirena ya sonará. Llama al 112. |
| Intrusión | No intervengas. La alarma suena y avisa automáticamente. |
| Alarma sonando sin motivo | Introduce PIN en la pantalla de alerta para silenciarla. |
| Sin agua después de una alerta | Pantalla RELÉS → panel verde AGUA → pulsar para abrir. |
