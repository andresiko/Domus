# DOMUS — Integración Home Assistant

Control remoto y notificaciones push para el sistema DOMUS via Home Assistant en Raspberry Pi, acceso externo con Cloudflare Tunnel (gratis, sin IP pública).

---

## Arquitectura

```
Móvil (app HA) ──HTTPS──▶ Cloudflare Tunnel ──▶ Raspberry Pi (HA)
                                                         │  MQTT
                                          ┌──────────────┤
                                     CrowPanel        A6v3
                                   (broker 1883)   (relés/sensores)
```

HA se conecta al broker MQTT que ya corre en la CrowPanel. No hace falta tocar ni la CrowPanel ni la A6v3.

---

## Paso 1 — Instalar Home Assistant OS en la Pi

1. Descarga **Raspberry Pi Imager**: https://www.raspberrypi.com/software/
2. Inserta la SD (mínimo 8 GB, recomendado 32 GB clase A1)
3. En Imager: **Choose OS → Other specific-purpose OS → Home Assistants and home automation → Home Assistant**
   - Elige la versión para Raspberry Pi 3 (64-bit)
4. Escribe la imagen en la SD (sin configuración adicional, HA se configura por web)
5. Inserta la SD en la Pi, conecta cable Ethernet al router y enciende
6. Espera 5-10 minutos (primer arranque descarga actualizaciones)
7. Abre en el navegador: **http://homeassistant.local:8123**
   - Si no funciona, busca la IP de la Pi en tu router y usa http://192.168.1.XXX:8123
8. Crea la cuenta de administrador cuando te lo pida

---

## Paso 2 — Configurar MQTT

La CrowPanel actúa de broker MQTT en el puerto 1883 sin autenticación.

1. En HA: **Settings → Devices & Services → Add Integration → MQTT**
2. Rellena:
   - **Broker**: IP de la CrowPanel (búscala en tu router, es el ESP32 con hostname `crowpanel` o similar)
   - **Port**: 1883
   - **Username**: (vacío)
   - **Password**: (vacío)
3. Clic en **Submit** → HA debería confirmar conexión

---

## Paso 3 — Instalar los archivos de configuración DOMUS

Necesitas acceso SSH o al editor de archivos de HA (addon **File Editor** o **Studio Code Server**).

### Instalar File Editor (más fácil):
1. **Settings → Add-ons → Add-on Store → File editor** → Install → Start
2. Activa "Show in sidebar"

### Editar configuration.yaml:
Abre `/config/configuration.yaml` y añade al final:
```yaml
homeassistant:
  packages: !include_dir_named packages
```

### Copiar los archivos DOMUS:
Crea las carpetas y copia los archivos de este repo:
- `ha/packages/domus.yaml` → `/config/packages/domus.yaml`
- `ha/automations/domus_alerts.yaml` → `/config/automations/domus_alerts.yaml`

Si ya tienes `automations.yaml`, abre el archivo y pega el contenido al final.

### Reiniciar HA:
**Settings → System → Restart**

Después de reiniciar verás las entidades en **Settings → Devices & Services → MQTT**.

---

## Paso 4 — Configurar las notificaciones

1. Instala **Home Assistant** en tu móvil (Android/iOS, es gratuita)
2. Ábrela y conéctate a tu HA local (http://homeassistant.local:8123)
3. En HA web: **Settings → Companion App** — verás tu móvil listado
4. Anota el nombre del servicio de notificación (será algo como `notify.mobile_app_pixel_8`)
5. Edita `/config/automations/domus_alerts.yaml` y reemplaza `notify.mobile_app_tu_movil` por ese nombre en todos los bloques
6. Reinicia HA o recarga las automatizaciones (**Settings → Automations → Reload**)

---

## Paso 5 — Acceso externo con Cloudflare Tunnel (gratis)

### Prerequisitos:
- Cuenta gratuita en https://cloudflare.com
- Un dominio (puedes registrar uno en Cloudflare Registrar desde ~8€/año, o usar un subdominio de un dominio que ya tengas)

### Instalar el addon Cloudflare Tunnel en HA:

1. **Settings → Add-ons → Add-on Store**
2. Clic en los 3 puntos (arriba derecha) → **Repositories** → añade:
   ```
   https://github.com/brenner-tobias/ha-addons
   ```
3. Busca **Cloudflare Tunnel** → Install
4. En la configuración del addon, añade tu `tunnel_token` (lo obtienes en el paso siguiente)

### Crear el túnel en Cloudflare:

1. Ve a https://one.dash.cloudflare.com → **Access → Tunnels → Create a tunnel**
2. Nombre: `domus-ha`
3. Instala el conector: elige **Docker** (pero no lo ejecutes, solo copia el token)
4. Copia el token largo que aparece tras `--token`
5. En HA, pega ese token en la configuración del addon Cloudflare Tunnel
6. En el mismo asistente de Cloudflare, en **Public Hostname**:
   - Subdomain: `domus` (o el que quieras)
   - Domain: tu dominio
   - Service: `http://homeassistant:8123`
7. Guarda — en 1-2 minutos `https://domus.tudominio.com` estará activo

### Conectar el móvil externamente:
En la app HA móvil: **Settings → Companion App → Home Assistant URL** → añade la URL externa `https://domus.tudominio.com`

---

## Entidades disponibles

| Entidad | Descripción |
|---------|-------------|
| `switch.agua` | Electroválvula agua (toggle) |
| `switch.sirena` | Sirena 12V (toggle) |
| `switch.calefaccion_relay` | Relay calefacción (solo lectura recomendada) |
| `binary_sensor.inundacion` | Sensor inundación (on=mojado) |
| `binary_sensor.movimiento` | PIR (on=detectado) |
| `binary_sensor.humo_calor` | Detector cocina (on=alarma) |
| `binary_sensor.red_electrica` | 220VAC (on=OK, off=corte) |
| `binary_sensor.alarma_activa` | Alarma armada o disparada |
| `binary_sensor.alarma_sonando` | Sirena activa por alarma |
| `sensor.temperatura_interior` | Temperatura sensor Tuya |
| `sensor.humedad` | Humedad sensor Tuya |
| `sensor.estado_alarma` | OFF / ARMING / ARMED / GRACE / SOUNDING |
| `sensor.modo_calefaccion` | OFF / Manual / Consigna / Programa |

---

## Tópicos MQTT de referencia

| Tópico | Dirección | Contenido |
|--------|-----------|-----------|
| `A6v3/30EDA03B1378/STATE` | A6v3 → todos | Estado inputs/outputs (retain=true) |
| `A6v3/30EDA03B1378/SET` | HA/CrowPanel → A6v3 | Comandos relay |
| `DOMUS/status` | CrowPanel → todos | Alarma, temperatura, humedad, modos |

---

## Notas

- **Sin Nabu Casa**: todo gratis. Nabu Casa (7€/mes) facilita la integración con Google Home por voz, pero no es necesaria para control remoto ni notificaciones.
- **Google Home por voz**: si en el futuro quieres control por voz, se puede añadir con la integración `google_assistant` local de HA (requiere proyecto en Google Cloud Console, gratuito pero 30 min de configuración).
- **Broker en la CrowPanel**: si la CrowPanel se reinicia, HA perderá la conexión MQTT unos segundos y la recuperará sola. Es estable en uso normal.
