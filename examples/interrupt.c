#include <pico/stdlib.h>
#include <stdio.h>

// Define the GPIO pin to monitor (e.g., GPIO 15)
#define INTERRUPT_PIN 15

// Interrupt callback function
void gpio_callback(uint gpio, uint32_t events) {
  // Check if the interrupt was triggered by a falling edge on our pin
  if (gpio == INTERRUPT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
    printf("Interrupt triggered: Pin %d went low!\n", gpio);
    // Add your interrupt handling code here
  }
}

int main() {
  // Initialize standard I/O
  stdio_init_all();

  // Configure the GPIO pin as input with a pull-up resistor
  gpio_init(INTERRUPT_PIN);
  gpio_set_dir(INTERRUPT_PIN, GPIO_IN);
  gpio_pull_up(INTERRUPT_PIN); // Enable pull-up to ensure a defined state

  // Set up the interrupt
  gpio_set_irq_enabled_with_callback(
      INTERRUPT_PIN,      // GPIO pin
      GPIO_IRQ_EDGE_FALL, // Trigger on falling edge
      true,               // Enable the interrupt
      &gpio_callback      // Callback function
  );

  printf("Interrupt setup complete. Waiting for pin %d to go low...\n", INTERRUPT_PIN);

  // Main loop (can be empty as work is done in the interrupt handler)
  while (true) {
    // Optional: Add other tasks or sleep to save power
    tight_loop_contents();
  }

  return 0;
}