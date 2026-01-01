#pragma once

char mode_string[10]="pos";
char aux_string[10]="XY";

LiquidLine lineMode(8, 1, mode_string);
LiquidLine lineAux(12, 1, aux_string);

LiquidLine lineX(0, 0, "X=", effector_position_orientation.X);
lineX.set_decimalPlaces(2);
LiquidLine lineY(8, 0, "Y=", effector_position_orientation.Y);
lineY.set_decimalPlaces(2);
LiquidLine lineZ(0, 1, "Z=", effector_position_orientation.Z);
lineZ.set_decimalPlaces(2);
LiquidScreen screenXYZ(lineX, lineY, lineZ);
screenXYZ.add_line(lineMode);
screenXYZ.add_line(lineAux);


LiquidLine liner(0, 0, "r=", effector_position_orientation.r);
liner.set_decimalPlaces(2);
LiquidLine linep(8, 0, "p=", effector_position_orientation.p);
linep.set_decimalPlaces(2);
LiquidLine liney(0, 1, "y=", effector_position_orientation.y);
liney.set_decimalPlaces(2);
LiquidScreen screenRPY(liner, linep, liney);
screenRPY.add_line(lineMode);
screenRPY.add_line(lineAux);

LiquidLine linet1(0, 0, "t1=", thetas(0));
linet1.set_decimalPlaces(2);
LiquidLine linet2(8, 0, "t2=", thetas(1));
linet2.set_decimalPlaces(2);
LiquidLine linet3(0, 1, "t3=", thetas(2));
linet3.set_decimalPlaces(2);
LiquidLine linet4(0, 0, "t4=", thetas(3));
linet4.set_decimalPlaces(2);
LiquidLine linet5(8, 0, "t5=", thetas(4));
linet5.set_decimalPlaces(2);
LiquidLine linet6(0, 1, "t6=", thetas(5));
linet6.set_decimalPlaces(2);
LiquidScreen screenThetas1(linet1, linet2, linet3);
LiquidScreen screenThetas2(linet4, linet5, linet6);
screenThetas1.add_line(lineMode);
screenThetas1.add_line(lineAux);
screenThetas2.add_line(lineMode);
screenThetas2.add_line(lineAux);

LiquidMenu menu(lcd);
menu.init();
menu.add_screen(screenXYZ);
menu.add_screen(screenRPY);
menu.add_screen(screenThetas1);
menu.add_screen(screenThetas2);