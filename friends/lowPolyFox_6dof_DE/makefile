
TARGET := lowPolyFox_6dof_DE

$(TARGET): $(TARGET).d
	dmd -c Oscillator.d -fPIC
	dmd -c params.d -fPIC
	dmd -c DEforOscillator2.d -fPIC
	dmd -c japariSDK/japarilib.d -fPIC
	dmd -c $(TARGET).d -fPIC -J.
	dmd -of$(TARGET).friends $(TARGET).o japarilib.o Oscillator.o params.o DEforOscillator2.o -shared -defaultlib=libphobos2.so -L-undefined -Ldynamic_lookup
