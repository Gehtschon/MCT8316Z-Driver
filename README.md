# English
German text is available after the the English section
### Structure

Communication between the MCU and the MCT8316Z takes place via SPI. The STM HAL library was used for communication via SPI. This controls the communication with
the set parameters such as speed, data word width and bit order for the SPI bus. A struct with the name MCT8316 was created in the driver. This struct contains
fields for the SPI bus used and the corresponding chip select pin. These fields are
written with the init function. The ten registers of the MCT8316 are also stored in this struct
are also stored in this struct. These registers were created as unions, which contain a struct. The struct
contains the individual bits as readable names. This means that they can be written in a readable form. The entire register can be displayed or written with the uint8_t variable named data
or described.

``` {.objectivec language="C"}
typedef union {
 struct {
	bool DRV_OFF :1; // Driver Off Bit
	bool OCP_CBC :1; // OCP PWM Cycle Operation Bit
	uint8_t OCP_DEG :2; // OCP Deglitch Time Settings
	bool OCP_RETRY :1; // OCP Retry Time Settings
	bool OCP_LVL :1; // Overcurrent Level Setting
	uint8_t OCP_MODE :2; // OCP Fault Options
} fields;
 uint8_t data;
} IC_Control_Register4;
```

| Field | Bits |
|-----------|----------|
| DRV_OFF | 7 |
| OCP_CBC | 6 |
| OCP_DEG | 5, 4 |
| OCP_RETRY | 3 |
| OCP_LVL | 2 |
| OCP_MODE | 1, 0 |


The code snippet corresponds to Table 4.9, which in turn corresponds to control register 4 of the driver chip. The table clearly shows that the fields and the variable Data are in the same memory area.
are located in the same memory area.
The MCT8316 struct ultimately has the structure from Table 4.10


| Name | Type | Description |
|------------|------------------------|----------------------------------------------------|
| spiHandle | SPI_HandleTypeDef | Pointer to the SPI instance to be used |
| NSS_Port | GPIO_TypeDef | Pointer to the port of the Chipselect pin |
| NSS_Pin | uint16_t | Chipselect pin number |
| statReg | IC_Status_Register | Information from status register 0 |
| statReg1 | IC_Status_Register1 | Information from status register 1 |
| statReg2 | IC_Status_Register2 | Information from status register 2 |
| ctrlReg1 | IC_Control_Register1 | Information from control register 1 |
| ctrlReg2 | IC_Control_Register2 | Information from control register 2 |
| ctrlReg3 | IC_Control_Register3 | Information from control register 3 |
| ctrlReg4 | IC_Control_Register4 | Information from control register 4 |
| ctrlReg5 | IC_Control_Register5 | Information from control register 5 |
| ctrlReg6 | IC_Control_Register6 | Information from control register 6 |
| ctrlReg7 | IC_Control_Register7 | Information from control register 7 |
| ctrlReg8 | IC_Control_Register8 | Information from control register 8 |
| ctrlReg9 | IC_Control_Register9 | Information from control register 9 |
| ctrlReg10 | IC_Control_Register10 | Information from control register 10 |

### MCT8316 Initialization
The initialization of the motor driver chip must be adapted to the specific project.For the
used PCB in combination with the Maxon ECXSP16, the following parameters are set

- Deactivate buck converter

- Activate braking mode

- Slew rate to 200V/us. This sets the speed at which the
    [MOSFET]{acronym-label="MOSFET" acronym-form="singular+short"}'s
    can switch between 20% and 80% voltage.

- PWM]{acronym-label="PWM" acronym-form="singular+short"} Mode to
    digital Hall sensors asynchronous motor (suitable for use with
    Maxon motor)

- Motor Lock Detection 5000mS and Motor Lock no report.These
    settings are required for very low speeds and are changed dynamically during
    changed dynamically during operation. See section**

These are the start settings, certain parameters are adjusted during operation as described above.
described above during operation.

### MCT8316_Write

The prototype of this function looks as follows:
HAL_StatusTypeDef MCT8316_Write (MCT8316 ∗dev , uin t8_ t ∗ add re s s ,
uin t8_ t ∗data , uin t8_ t ∗ oldData )
The MCT8316_Write function requires the following transfer parameters.
- Device
- Address
- oldData
- oldData
The data is written to the transferred address. As a return value, the chip sends the
previously set data. If writing and reading was successful, the function HAL_OK
returns.

### MCT8316_Write_External

To simplify writing from outside the driver software, the following function has been added to the
function MCT8316_Write, the function MCT8316_Write_Extern was created. The prototype of this
function looks as follows:
uin t8_ t MCT8316_Write_Extern (MCT8316 ∗dev , uin t8_ t add re s sp ,
uin t8_ t ∗ data )
The MCT8316_Write function requires the following transfer parameters.
- Device
- Device address
- Data
The fundamental difference to the MCT8316_Write function is the error handling of this function. If this function is used, the fault bits are deleted before the write command.
The deletion prevents error messages that are not current from being read out and
lead to an error. If the write error worked, the return value of the function is
0, otherwise a number greater than 0 (details are explained in the code)

### MCT8316_Read

The prototype of this function looks like this:
HAL_StatusTypeDef MCT8316_Read(MCT8316 ∗dev , uin t8_ t ∗ add re s s ,
uin t8_ t ∗ data )
The MCT8316_Read function requires the following transfer parameters.
- Device
- Address
- Data
The read values are saved to Data. If the function was successful, this is confirmed with
confirmed with a HAL_OK.

### MCT8316 statusRegisterFault

The prototype of this function looks as follows:
uin t8_ t s t a t u s R e g i s t e r F a u l t (MCT8316 ∗dev )
This function checks whether there is an error on the MCT8316. The return value is structured as follows
is structured as follows:
- Status register has an error: 1
- Status register1 has an error: 10
- Status register 2 has an error: 100
- No error: 0
The return value can also be a combination of the numbers if there are several errors.

**Example** Status register1 and status register2 have an error.
have an error. The return value is 110.

### parity_calc

The prototype of this function looks like this:

``` {.objectivec language="C"}
unsigned char parity_calc(unsigned char value)
```

This function is used purely internally in the driver. It calculates the
parity bit of the transfer value. This function is required for
SPI bus communication with the MCT8316 to calculate the parity bit. The
parity bit is set so that the data word (16 bits) contains an even number of
number of ones and zeros. The table
shows the systematic structure of the word.
can be seen in the table.

The code snippet shows the function that calculates the parity bit
is calculated. First, the transfer value is stored in the variable parity
variable. Subsequently, an exclusive-or operation
(XOR) and bit-shifting to determine the parity bit of the first 2 bits.
This process is repeated for the first 4 bits, then for all 8 bits.
all 8 bits. The result is stored in the least significant bit.
The truth table of the XOR operation is shown in the table.

| A | B | Y |
|-----|-----|-----|
| 0 | 0 | 0 |
| 0 | 1 | 1 |
| 1 | 0 | 1 |
| 1 | 1 | 0 |

::: listing
``` c
unsigned char parity_calc(unsigned char value) {
	// Initialize a variable to hold the parity value
	unsigned char parity = value;
	// XOR value with itself shifted right by 1 bit to get
    //the parity of the first 2 bits
	parity = parity ^ (value >> 1);
	// XOR the result with itself shifted right by 2 bits to get
//the parity of the first 4 bits
	parity = parity ^ (parity >> 2);
	// XOR the result with itself shifted right by 4 bits to get
// the parity of all 8 bits
	parity = parity ^ (parity >> 4);
	// The parity is the least significant bit of the result
	return parity & 1;
}
```
:::

In the MCT8316 driver, the parity bit of the first byte is calculated first.
is calculated, which consists of the read/write bit and the address.
The parity bit is then calculated from the second byte, which consists of the payload
is used to calculate the parity bit. The code is shown in the code snippet
is shown. After the two parity bits have been calculated separately,
they are offset by an XOR operation, which produces the overall result.
result. The calculated result corresponds to the parity bit of the
data word. The data word is formed in the last two lines of the
code snippet.

::: listing
``` c
	// Calculate the parity of the data
	unsigned char parity_1 = parity_calc((rw << 7) | (*address << 1));
	unsigned char parity_2 = parity_calc(*data);

	// the parity is for the whole
	unsigned char parity = parity_1 ^ parity_2;

    // First byte: R/W, address, parity
    pTxData[0] = (rw << 7) | (*address << 1) | (parity); 
    pTxData[1] = *data; // Second byte: data
```
:::

### checkRegisterOnValue 
The prototype of this function looks like this:
uin t8_ t checkRe gi s te rOnV alue (MCT8316 ∗dev , uin t8_ t reg , uin t8_ t v al u e )
The checkRegisterOnValue function requires the following transfer parameters.
- Device
- The register to be checked
- The value to be checked
This function checks whether a specific value occurs in a specific register (reg).
occurs. If this is the case, a 1 is returned, otherwise a 0. If the register does not exist, 99 is returned. The register that is to be checked is linked bit by bit with
a specific mask bit by bit. The mask excludes bits that contain a random,
undefined value. It is IMPORTANT that the register to be checked is first linked with the
MCT8316_Read function, this is not part of this function.
Table 4.13 contains an example of the bit mask. The existing register
has a value at bit 5, which is not of interest. The mask is ANDed with the value
of the register AND-linked. As bit 5 of the mask is 0, the result at bit 5 will always be
will always contain a 0. This means that the result can be analyzed without a randomly dependent
value occurs in the register

### MCT8316_Fault_Clear 

The prototype of this function looks as follows:
uin t8_ t MCT8316_Fault_Clear (MCT8316 ∗dev )
This function resets the error register of the MCT8316. If the reset was successful
was successful, a 0 is returned, otherwise a 1.

#German / Deutsch
### Aufbau

Die Kommunikation zwischen der MCU und dem MCT8316Z erfolgt per SPI. Für die Kommunikation per SPI wurde die STM HAL-Library verwendet. Diese regelt die Kommunikation mit
den eingestellten Parameter wie Geschwindigkeit, Datenwortbreite und Bit-Order für den SPIBus. Im Treiber wurde ein Struct mit dem Namen MCT8316 erstellt. Dieses Struct beinhaltet
Felder für den benutzen SPI-Bus und den dazugehörigen Chipselect Pin. Diese Felder werden
mit der init-Funktion beschrieben. Die zehn Register des MCT8316 sind auch in diesem Struct
abgelegt. Diese Register wurden als Unions erstellt, welche ein Struct beinhalten. Das Struct
beinhaltet die einzelnen Bits als lesbare Namen. Somit können diese in einer lesbaren Form beschrieben werden. Das gesamte Register kann mit der uint8_t Variable namens data dargestellt
oder beschrieben werden.

``` {.objectivec language="C"}
typedef union {
 struct {
	bool DRV_OFF :1; // Driver Off Bit
	bool OCP_CBC :1; // OCP PWM Cycle Operation Bit
	uint8_t OCP_DEG :2; // OCP Deglitch Time Settings
	bool OCP_RETRY :1; // OCP Retry Time Settings
	bool OCP_LVL :1; // Overcurrent Level Setting
	uint8_t OCP_MODE :2; // OCP Fault Options
} fields;
 uint8_t data;
} IC_Control_Register4;
```

| Field     | Bits     |
|-----------|----------|
| DRV_OFF   | 7        |
| OCP_CBC   | 6        |
| OCP_DEG   | 5, 4     |
| OCP_RETRY | 3        |
| OCP_LVL   | 2        |
| OCP_MODE  | 1, 0     |


Das Codesnippet entspricht der Tabelle 4.9, welche wiederum dem Controlregister 4 des Treiberchips entspricht. In der Tabelle ist gut zu sehen, dass die Fields und die Variable Data auf
dem gleichen Speicherbereich liegen.
Das MCT8316 Struct hat schlussendlich den Aufbau aus Tabelle 4.10


| Name       | Typ                    | Beschreibung                                       |
|------------|------------------------|----------------------------------------------------|
| spiHandle  | SPI_HandleTypeDef     | Pointer to the SPI instance to be used             |
| NSS_Port   | GPIO_TypeDef           | Pointer to the port of the Chipselect pin          |
| NSS_Pin    | uint16_t               | Chipselect pin number                              |
| statReg    | IC_Status_Register     | Information from status register 0                  |
| statReg1   | IC_Status_Register1    | Information from status register 1                  |
| statReg2   | IC_Status_Register2    | Information from status register 2                  |
| ctrlReg1   | IC_Control_Register1   | Information from control register 1                 |
| ctrlReg2   | IC_Control_Register2   | Information from control register 2                 |
| ctrlReg3   | IC_Control_Register3   | Information from control register 3                 |
| ctrlReg4   | IC_Control_Register4   | Information from control register 4                 |
| ctrlReg5   | IC_Control_Register5   | Information from control register 5                 |
| ctrlReg6   | IC_Control_Register6   | Information from control register 6                 |
| ctrlReg7   | IC_Control_Register7   | Information from control register 7                 |
| ctrlReg8   | IC_Control_Register8   | Information from control register 8                 |
| ctrlReg9   | IC_Control_Register9   | Information from control register 9                 |
| ctrlReg10  | IC_Control_Register10  | Information from control register 10                |


### MCT8316 Initialisation

Die Initialisierung des Motorentreiberchips muss projektspezifisch angepasst werden. Für das
benutzte PCB in Kombination mit dem Maxon ECXSP16, werden folgende Parameter gesetzt

-   Buck Converter deaktivieren

-   Bremsmodus aktivieren

-   Slew Rate auf 200V/us. Dies setzt die Geschwindigkeit, in der die
    [MOSFET]{acronym-label="MOSFET" acronym-form="singular+short"}'s
    zwischen 20 % und 80% Spannung schalten können.

-   [PWM]{acronym-label="PWM" acronym-form="singular+short"} Modus zu
    digitalen Hallsensoren Asynchron Motor (Passend zum Benutzen
    Maxon-Motor)

-   Motor Lock Detection 5000mS und Motor Lock kein Report. Diese
    Einstellungen sind für sehr kleine Drehzahlen nötig und wird während
    dem Betriebe dynamisch geändert. Siehe Abschnitt**

Dies sind die Starteinstellungen, gewisse Parameter werden wie oben
beschrieben während des Betriebes angepasst.

### MCT8316_Write

Der Prototyp dieser Funktion sieht wie folgt aus:
HAL_StatusTypeDef MCT8316_Write (MCT8316 ∗dev , uin t8_ t ∗ add re s s ,
uin t8_ t ∗data , uin t8_ t ∗ oldData )
Die Funktion MCT8316_Write benötigt folgende Übergabeparameter.
• Device
• Adresse
• Data
• oldData
Die Daten werden auf die übergebene Adresse geschrieben. Als Rückgabewert sendet der Chip die
vorher gesetzten Daten. War das Schreiben und Lesen erfolgreich, gibt die Funktion HAL_OK
zurück.

### MCT8316_Write_Extern

Um das Beschreiben von ausserhalb der Treibersoftware zu vereinfachen, wurde zusätzlich zur
Funktion MCT8316_Write, die Funktion MCT8316_Write_Extern erstellt. Der Prototyp dieser
Funktion sieht wie folgt aus:
uin t8_ t MCT8316_Write_Extern (MCT8316 ∗dev , uin t8_ t add re s sp ,
uin t8_ t ∗ data )
Die Funktion MCT8316_Write benötigt folgende Übergabeparameter.
• Device
• Adresse
• Data
Der grundlegende Unterschied zur Funktion MCT8316_Write ist das Errorhandling dieser Funktion. Wenn diese Funktion verwendet wird, werden die Fault Bits vor dem Schreibbefehl gelöscht.
Mit dem Löschen wird verhindert, dass nicht aktuelle Fehlermeldungen ausgelesen werden und
zu einem Fehler führen. Wenn der Schreibfehler funktionierte, ist der Rückgabewert der Funktion
0, ansonsten eine Zahl grösser 0. (Details sind im Code erläutert)

### MCT8316_Read

Der Prototyp dieser Funktion sieht wie folgt aus:
HAL_StatusTypeDef MCT8316_Read(MCT8316 ∗dev , uin t8_ t ∗ add re s s ,
uin t8_ t ∗ data )
Die Funktion MCT8316_Read benötigt folgende Übergabeparameter.
• Device
• Adresse
• Data
Die gelesenen Werte werden auf Data abgespeichert. War die Funktion erfolgreich, wird dies mit
einem HAL_OK bestätigt.

### MCT8316 statusRegisterFault

Der Prototyp dieser Funktion sieht wie folgt aus:
uin t8_ t s t a t u s R e g i s t e r F a u l t (MCT8316 ∗dev )
Diese Funktion prüft, ob ein Fehler auf dem MCT8316 vorliegt. Der Rückgabewert ist wie folgt
aufgebaut:
• Statusregister hat einen Fehler: 1
• Statusregister1 hat einen Fehler: 10
• Statusregister2 hat einen Fehler: 100
• Kein Fehler: 0
Der Rückgabewert kann auch eine Kombination der Zahlen sein, falls mehrere Fehler vorliegen.

**Beispiel** Statusregister1 und Statusregister2 weisen einen Fehler
auf. Der Rückgabewert beträgt 110.

### parity_calc

Der Prototyp dieser Funktion sieht wie folgt aus:

``` {.objectivec language="C"}
unsigned char parity_calc(unsigned char value)
```

Diese Funktion wird rein intern im Treiber benutzt. Sie berechnet das
Paritätsbit des Übergabewertes. Diese Funktion wird benötigt, um bei der
SPI-Bus Kommunikation, mit dem MCT8316, das Paritätsbit zu berechnen. Das
Paritätsbit wird so gesetzt, dass im Datenwort (16 Bit's) eine gerade
Anzahl von Einsen und Nullen vorkommt. In der Tabelle
ist der systematische Aufbau des Wortes zu
sehen.

Im Code-Snippet ist die Funktion zu sehen, welche das Paritätsbit
berechnet. Als Erstes wird der Übergabewert in die Variabel parity
abgespeichert. Anschliessend wird durch eine Exklusiv-Oder Verknüpfung
(XOR) und Bit-shifting das Paritätsbit der ersten 2 Bits bestimmt.
Dieser Vorgang wird für die ersten 4 Bits wiederholt, anschliessend für
alle 8 Bits. Das Resultat ist im wenigstens signifikanten Bit abgelegt.
Die Wahrheitstabelle der XOR Verknüpfung ist in Tabelle dargestellt.

| A   | B   | Y   |
|-----|-----|-----|
| 0   | 0   | 0   |
| 0   | 1   | 1   |
| 1   | 0   | 1   |
| 1   | 1   | 0   |


::: listing
``` c
unsigned char parity_calc(unsigned char value) {
	// Initialize a variable to hold the parity value
	unsigned char parity = value;

	// XOR value with itself shifted right by 1 bit to get
    //the parity of the first 2 bits
	parity = parity ^ (value >> 1);
	// XOR the result with itself shifted right by 2 bits to get
    //the parity of the first 4 bits
	parity = parity ^ (parity >> 2);
	// XOR the result with itself shifted right by 4 bits to get
    // the parity of all 8 bits
	parity = parity ^ (parity >> 4);

	// The parity is the least significant bit of the result
	return parity & 1;
}
```
:::

Im MCT8316 Treiber wird zuerst das Paritätsbit des ersten Byte
berechnet, welches aus dem Read/Write Bit und der Adresse besteht.
Anschliessend wird aus dem zweiten Byte, welches aus dem Payload
besteht, das Paritätsbit berechnet. Der Code ist im Code-Snippet
dargestellt. Nachdem die beiden Paritätsbits separat berechnet wurden,
werden sie durch eine XOR Verknüpfung verrechnet, was zum Gesamtresultat
führt. Das berechnete Resultat entspricht dem Paritätsbit des
Datenwortes. Das Datenwort wird in den letzten beiden Zeilen des
Codesnippets gebildete.

::: listing
``` c
	// Calculate the parity of the data
	unsigned char parity_1 = parity_calc((rw << 7) | (*address << 1));
	unsigned char parity_2 = parity_calc(*data);

	// the parity is for the whole
	unsigned char parity = parity_1 ^ parity_2;

    // First byte: R/W, address, parity
    pTxData[0] = (rw << 7) | (*address << 1) | (parity); 
    pTxData[1] = *data; // Second byte: data
```
:::

### checkRegisterOnValue 
Der Prototyp dieser Funktion sieht wie folgt aus:
uin t8_ t checkRe gi s te rOnV alue (MCT8316 ∗dev , uin t8_ t reg , uin t8_ t v al u e )
Die Funktion checkRegisterOnValue benötigt folgende Übergabeparameter.
• Device
• Das zu prüfendes Register
• Der zu prüfende Wert
Diese Funktion überprüft, ob ein bestimmter Wert (value) in einem bestimmten Register (reg)
auftritt. Falls dies der Fall ist, wird eine 1 zurückgegeben, andernfalls eine 0. Wenn das Register nicht existiert, wird 99 zurückgegeben. Das Register, das überprüft werden soll, wird mit
einer spezifischen Maske bitweise verknüpft. Die Maske exkludiert Bits, welche einen zufälligen,
nicht definierten Wert enthalten. WICHTIG ist, dass zuerst das zu prüfende Register mit der
MCT8316_Read Funktion gelesen werden muss, dies ist nicht Bestandteil dieser Funktion.
In der Tabelle 4.13 ist ein Beispiel zu der Bit Maske vorhanden. Das vorhandene Register
besitzt einen Wert bei Bit 5, welcher nicht von Interesse ist. Die Maske wird mit dem Wert
des Registers UND-Verknüpft. Da das Bit 5 der Maske 0 ist, wird im Resultat bei Bit 5 immer
eine 0 stehen. Somit kann das Resultat analysiert werden, ohne dass ein vom Zufall abhängiger
Wert im Register vorkommt

### MCT8316_Fault_Clear 

Der Prototyp dieser Funktion sieht wie folgt aus:
uin t8_ t MCT8316_Fault_Clear (MCT8316 ∗dev )
Diese Funktion setzt das Fehlerregister des MCT8316 zurück. Wenn das Rücksetzten erfolgreich
war, wird eine 0 zurückgegeben, ansonsten eine 1.

nicht verarbeitet werden konnten. Mit der gedrosselten Geschwindigkeit
passierte dies nicht.
