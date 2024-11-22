## Instalación y configuración

### MCUXpresso IDE
    Una IDE o entorno de desarrollo integrado es un sistema de software para el diseño de aplicaciones que combina herramientas del desarrollador comunes en una sola interfaz gráfica de usuario. Generalmente un IDE cuenta con las siguientes características:

- **Editor de código fuente:** Editor de texto que ayuda a escribir el código con funciones como el resaltado de la sintaxis con indicaciones visuales, el relleno automático específico para el lenguaje y la comprobación de errores a medida que se escribe el código.

- **Automatización de las compilaciones locales:** Herramientas que automatizan las tareas sencillas y repetitivas como la compilación del código fuente en un archivo binario.

- **Depurador:** Software para probar los programas y mostrar la ejecución paso a paso para facilitar el debug.

#### Instalación
    Para la instalación de esta IDE será necesario descargar el instalador de la página oficial de NXP. Para esto dirigirse al siguiente link, registrarse e ingresar a la sección de descargas.

[MCUXpresso IDE for NXP MCUs | Linux, Windows and MacOS | NXP Semiconductors | NXP Semiconductors](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)

    Se deberán aceptar los terminos y condiciones y luego nos llevará a una ventana con todos los instaladores disponibles. Descargar el adecuado para su sistema operativo (Linux, macOS, Windows). En caso de tener Windows como sistema operativo, el instalador que se descarga es un .exe que simplemente hay que ejecutar y seguir las instrucciones de instalación.
   
    Al ejecutar la aplicación, lo primero que nos aparecerá será la elección de un workspace, este espacio de trabajo es el directorio principal en el que se almacenarán y organizarán los proyector y archivos relacionados con el desarrollo. Es el área en la que se trabaja con los proyectos y la IDE guarda los archivos de configuración y datos asociados. 

    El concepto de espacio de trabajo es útil porque te permite mantener tus proyectos organizados y separados. Puedes trabajar en múltiples proyectos en paralelo y cambiar fácilmente entre ellos dentro de la IDE. Además, el espacio de trabajo almacena las preferencias y configuraciones de la IDE, como el esquema de color, atajos de teclado y configuraciones del depurador, entre otras cosas.

    Esta ruta puede dejarse por defecto o se puede colocar otra.

    Al dar click en lanzar, se nos abrirá la ventana principal de la IDE. Para compilar este proyecto se debe generar un nuevo proyecto desde `file>new>create a new C/C++ project`. En el SDK wizard, se debe seleccionar la placa LPC 1769. Luego, elegir la opción `C project`, Elegir el nombre y ubicación del proyecto. Posteriormente se nos solicita que importemos las librerías a usar. Para esto, hacer click en `import`, dar click en `Project archive (zip) > Browse` y allí seleccionar el .zip que se descarga del drive ([CMSISv2p00_LPC17xx.zip](https://drive.google.com/drive/folders/10A9xhIxx6ag75GtEwLzxr8pCdP6hR1HC )). Una vez importada, seleccionarla como librería del proyecto. Esta librería proporciona una capa de abstracción de software que facilita el desarrolllo de software embebido para microcontroladores basados en la arquitectura ARM Cortex-M. Ademas seleccionar la libreria CMSISV2p00 antes de `continuar` y en las librerías de DSP no es necesario seleccionar nada, simplemente dar a siguiente.

    Por último, nos da la opción de elegir la estructura de directorios del proyecto. Dandonos la posibilidad de cambiar el nombre del directorio para los fuentes y la posibilidad de crear un directorio 'inc' para los headers file y añadir automáticamente al path este mismo. Dar en `Finish` y se abrirá la ventana de un proyecto con un main básico, alli copiar el codigo de la carpeta `src` de este repositorio y compilar.

### PuTTy
Para poder interactuar con el sistema en control manual tambien es necesario instalar Putty, una implementación gratuita de SSH y Telnet para Windows y Unix plataformas, junto con un emulador de terminal. [Link de Descarga](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html).
La configuracion de la interfaz depende del puerto de cada computadora lo cual se puede verificar desde `Administrado de dispositivos` en el cual una vez conectado el USB UART se identifica el puerto al que esta conectado que es la información que debe colocarse en el emulador y seleccionar comunicación serial.
