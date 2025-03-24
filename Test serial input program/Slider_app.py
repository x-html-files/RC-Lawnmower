import tkinter as tk
import serial
import threading

class SerialSliderApp:
    def __init__(self, port='COM6', baudrate=115200):
        self.root = tk.Tk()
        self.root.title("Serial Slider Control")
        
        # Configure serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
        
        # Create slider
        self.slider = tk.Scale(
            self.root, 
            from_=0, 
            to=350, 
            orient=tk.HORIZONTAL, 
            length=300, 
            command=self.on_slider_move
        )
        self.slider.pack(padx=20, pady=20)
        
        # Value display label
        self.value_label = tk.Label(self.root, text="Value: 0")
        self.value_label.pack()
        
        # Serial input field
        self.input_label = tk.Label(self.root, text="Serial Input:")
        self.input_label.pack()
        self.input_text = tk.Text(self.root, height=5, width=40)
        self.input_text.pack(padx=20, pady=10)
        
        # Start serial reading thread
        if self.ser:
            self.reading_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.reading_thread.start()
        
    def on_slider_move(self, value):
        value = int(float(value))
        self.value_label.config(text=f"Value: {value}")
        
        # Send value via serial if connection exists
        if self.ser:
            try:
                self.ser.write(f"{value}\n".encode())
            except Exception as e:
                print(f"Serial write error: {e}")
        
    def read_serial(self):
        while True:
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('utf-8').strip()
                    self.root.after(0, self.update_input_text, data)
            except Exception as e:
                print(f"Serial read error: {e}")
                break
    
    def update_input_text(self, data):
        self.input_text.insert(tk.END, data + '\n')
        self.input_text.see(tk.END)
        
    def run(self):
        self.root.mainloop()
        
        # Close serial port when app closes
        if self.ser:
            self.ser.close()

# Run the application
if __name__ == "__main__":
    app = SerialSliderApp()
    app.run()