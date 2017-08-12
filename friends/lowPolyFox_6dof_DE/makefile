
TARGET := lowPolyFox_6dof_DE

$(TARGET): $(TARGET).d
	dmd -c agent.d -fPIC
	dmd -c Oscillator.d -fPIC
	dmd -c params.d -fPIC
	dmd -c DEforOscillator2.d -fPIC
	dmd -c loadJson.d -fPIC -J.
	dmd -c japariSDK/japarilib.d -fPIC
	dmd -c $(TARGET).d -fPIC
	dmd -of$(TARGET).friends $(TARGET).o agent.o japarilib.o Oscillator.o params.o DEforOscillator2.o loadJson.o -shared -defaultlib=libphobos2.so -L-undefined -Ldynamic_lookup
