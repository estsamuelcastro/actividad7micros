# ACTIVIDAD 7 MICROS - SAMUEL CASTRO

## Resultado final

El proyecto implementa una colonia distribuida de tres nodos ACO:

- Debido a que la actividad exigía la implementación de tres placas ESP 32, se tomó la alternativa de usar una física y dos virtuales ya que no se contaba con la facilidad de conseguir tres placas
Los dos ESP32 virtuales son una emulación de comportamiento del nodo, no una emulación electrónica del microcontrolador completo. Esta adaptación permite cumplir la arquitectura de tres nodos usando el hardware disponible.

Por ende se implementó una colonia distribuida de hormigas compuesta por un ESP32 físico y dos nodos virtuales. Cada nodo ejecuta ACO de manera independiente, calcula rutas hacia una meta y actualiza su matriz de feromonas. La información de feromonas y las mejores rutas se intercambia mediante UDP sobre la red Wi-Fi creada por el ESP32 físico. Finalmente, PyBullet funciona como gemelo digital de la colonia y representa los tres nodos como robots que recorren el laberinto siguiendo las rutas obtenidas. 
A continuacion se puede observar el comportamiento dentro de la esp 32 fisica, donde se evidencia las iteraciones del nodo 1; y despues se observa en la consola del sistema el mismo comportamiento pero en los demas nodos 2 y 3(esp32 virtuales).
<img width="1265" height="785" alt="image" src="https://github.com/user-attachments/assets/927e2bd4-7cf8-42ba-bf09-f74cf7e1870d" />
<img width="1771" height="861" alt="image" src="https://github.com/user-attachments/assets/e7578000-97ab-45d9-8495-981fc95a61c5" />

En este video se puede observar el funcionamiento de los tres "carritos", los cuales estan debidamente marcados con nodo 1, 2 y 3 para identificar cada uno.
https://drive.google.com/file/d/1uUTvye-1qK12f3uO72XI9gmeJJsALqTH/view?usp=sharing
