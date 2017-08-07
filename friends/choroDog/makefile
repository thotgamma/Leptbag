
TARGET := chorodog



$(TARGET): $(TARGET).d
	dmd -c GA.d -fPIC
	dmd -c japariSDK/japarilib.d -fPIC
	dmd -c $(TARGET).d -fPIC -J.
	dmd -of$(TARGET).friends $(TARGET).o japarilib.o GA.o -shared -defaultlib=libphobos2.so -L-undefined -Ldynamic_lookup
