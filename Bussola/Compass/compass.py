import py_qmc5883l
sensor = py_qmc5883l.QMC5883L()
#m = sensor.get_magnet()
#print(m)
sensor.declination=2.9
porta_avversaria = sensor.get_bearing()
print("heading angle: %d" %porta_avversaria)
while True :
	actual_position=sensor.get_bearing_raw()
	relative_position=actual_position #-porta_avversaria
	print(relative_position)


