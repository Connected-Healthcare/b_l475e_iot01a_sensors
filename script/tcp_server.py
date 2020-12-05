import socket

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)


def parse_and_print(dl: list):
    co_gas_concentration = dl[0]
    _, _ = dl[1], dl[2]
    sgp30_co2, sgp30_voc = dl[3], dl[4]

    hts221_temperature = dl[5]
    hts221_humidity = dl[6]
    lps22hb_temperature = dl[7]
    lps22hb_pressure = dl[8]

    magnetometer = list()
    magnetometer.append(data[9])
    magnetometer.append(data[10])
    magnetometer.append(data[11])

    acceleration = list()
    acceleration.append(data[12])
    acceleration.append(data[13])
    acceleration.append(data[14])

    gyroscope = list()
    gyroscope.append(data[15])
    gyroscope.append(data[16])
    gyroscope.append(data[17])

    time_of_flight = data[18]

    print(f'CO Gas Concentration {co_gas_concentration}')
    print(f'SGP30 CO2 {sgp30_co2}')
    print(f'SGP30 VOC {sgp30_voc}')

    print(f'HTS221_temperature {hts221_temperature}')
    print(f'HTS221_humidity {hts221_humidity}')
    print(f'Lps22hb_temperature {lps22hb_temperature}')
    print(f'Lps22hb_pressure {lps22hb_pressure}')
    print(f'Time_of_flight {time_of_flight}')
    print(
        f'Magnetometer {magnetometer[0]} {magnetometer[1]} {magnetometer[2]}')
    print(
        f'Acceleration {acceleration[0]} {acceleration[1]} {acceleration[2]}')
    print(
        f'Gyroscope {gyroscope[0]} {gyroscope[1]} {gyroscope[2]}')
    print('--------')


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            # print(f"recv: {data}")
            data = data.decode('utf-8')
            data = data.strip().split(',')
            parse_and_print(data)
