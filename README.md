# Maturaarbeit-Anhang
Ballbot-Code ist der Code, welcher benutzt wurde, um den Ballbot anzusteuern.
Beide Autotuner wurden nicht mehr verbessert, da entschlossen wurde die Ziegler-Nichols-Methode anzuwenden. Darum funktionieren beide nicht ganz. Autotune2 ist so angepasst, dass es vom Speicherplatz her für einen Arduino UnoRev3 funktioniert.
Sensor_Kalman ist der Code, welcher nur die Winkel herausgibt.
Motor_only ist der Code, welcher nur die Motoren ansteuern.
Der Kalman-Filter von Kristian Lauszus wurde leicht abgeändert.
Der Grossteil des Journals beinhaltet die Notizen, welche beim Designen des Gerüsts gemacht worden sind. Das Verhalten der Motoren und die Grundprinzipien des Ballbots sind auch 
aufgeschrieben. 
Im libraries-Ordner ist die selbst geschriebene Library. Um diese zu benutzen, muss man im Libraries-Ordner des Arduino-Dokumentes ein Ordner erstellen und die Library in diese hineinkopieren. Nun kann man wie gewöhnlich mit der #include-Funktion die Library aufrufen und benutzen.
MA ganz ist der Code für das Design des Gerüstes, aber da dieser zu gross für den Drucker war, musste das Gerüst geteilt werde. Folgende Dateien wurden gedruckt: MA Gerüst - linker Teil und rechter Teil, Chip-Halterungen und Omniwheel-Halterungen. Diese Codes müssen in OpenSCAD aufgerufen werden. 
