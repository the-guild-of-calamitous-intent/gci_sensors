# Linux I2C

![](./7-bit-address-i2c.gif)

```bash
i2cdetect -y 1
```

```c
ioctl(bus, I2C_SET_SPEED, 1500000)
ioctl(bus, I2C_SLAVE, PICAddress)
write(i2c,PICBytes,20)
```

- [Pololu i2c example](https://www.pololu.com/docs/0J73/15.8)

- [An Introduction to chardev GPIO and Libgpiod on the Raspberry PI](https://www.beyondlogic.org/an-introduction-to-chardev-gpio-and-libgpiod-on-the-raspberry-pi/)
  - Don't know, looks like access to `/sys/bus/gpio/devices/` to work with GPIO on pi ... unclear if you have to be root


- [pi projects, i2c example](https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface)

The 7 bit I2C address of all found devices will be shown (ignoring the R/W bit,
so I2C address `0000 0110` is displayed as hex `0x03`).

```c
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

int file_i2c;
int length;
unsigned char buffer[60] = {0};

//----- OPEN THE I2C BUS -----
char *filename = (char*)"/dev/i2c-1";
if ((file_i2c = open(filename, O_RDWR)) < 0)
{
    //ERROR HANDLING: you can check errno to see what went wrong
    printf("Failed to open the i2c bus");
    return;
}

int addr = 0x5a;          //<<<<<The I2C address of the slave
if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
{
    printf("Failed to acquire bus access and/or talk to slave.\n");
    //ERROR HANDLING; you can check errno to see what went wrong
    return;
}


//----- READ BYTES -----
length = 4;			//<<< Number of bytes to read
if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
{
    //ERROR HANDLING: i2c transaction failed
    printf("Failed to read from the i2c bus.\n");
}
else
{
    printf("Data read: %s\n", buffer);
}


//----- WRITE BYTES -----
buffer[0] = 0x01;
buffer[1] = 0x02;
length = 2;			//<<< Number of bytes to write
if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
{
    /* ERROR HANDLING: i2c transaction failed */
    printf("Failed to write to the i2c bus.\n");
}
```



## Maybe

https://docs.kernel.org/i2c/smbus-protocol.html

```c++
class TwoWire {
  public:
  TwoWire() {
    const char* device = "/dev/i2c-1"; // -0 is used for other stuff
    if ((fd = open (device, O_RDWR)) < 0) {
      printf("Fail open %s\n", device);
		}
  }

  // set handle as slave address
  void beginTransmission(uint8_t addr) {
    //addr = address;
    if (ioctl (fd, I2C_SLAVE, addr) < 0) {
      printf("write error\n");
      close(fd); // something is wrong, so stop?
    }
  }

  uint8_t endTransmission(bool sendStop) {
    // write
  }

  uint8_t write(uint8_t data) {
    // pack buffer
    i2c_smbus_write_byte_data
  }

  // how do I get reg into this???
  void readBlock(uint8_t reg, uint8_t length) {
    if (i2c_smbus_read_block_data(fd, reg, length, buffer) < 0) printf("block read error\n");
    // could also close fd on error and exit?
  }

  void readByte(uint8_t) {}

  protected:
  uint8_t buffer[32];
  //uint8_t addr; // dont' need to save??

};

```









