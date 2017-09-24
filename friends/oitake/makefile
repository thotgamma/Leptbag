
TARGET := app
model := oitake

$(TARGET): $(TARGET).d
	dmd -c agent.d -fPIC -J.
	dmd -c Oscillator.d -fPIC -J.
	dmd -c params.d -fPIC -J.
	dmd -c DEforOscillator2.d -fPIC -J.
	dmd -c DEforSOG.d -fPIC -J.
	dmd -c loadJson.d -fPIC -J.
	dmd -c japariSDK/japarilib.d -fPIC
	dmd -c $(TARGET).d -fPIC -J.
	dmd -of$(model).friends $(TARGET).o agent.o japarilib.o Oscillator.o params.o DEforOscillator2.o DEforSOG.o loadJson.o -shared -defaultlib=libphobos2.so -L-undefined -Ldynamic_lookup
