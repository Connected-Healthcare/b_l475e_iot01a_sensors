import socket

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)


def parse_and_print(dl: list):
    co_gas_concentration = dl[0]
    _, _ = dl[1], dl[2]

    hts221_temperature = dl[3]
    hts221_humidity = dl[4]
    lps22hb_temperature = dl[5]
    lps22hb_pressure = dl[6]

    magnetometer = list()
    magnetometer.append(data[7])
    magnetometer.append(data[8])
    magnetometer.append(data[9])

    acceleration = list()
    acceleration.append(data[10])
    acceleration.append(data[11])
    acceleration.append(data[12])

    gyroscope = list()
    gyroscope.append(data[13])
    gyroscope.append(data[14])
    gyroscope.append(data[15])

    time_of_flight = data[16]

    heartbeat = list()
    heartbeat.append(data[17])
    heartbeat.append(data[18])

    gps = list()
    gps.append(data[19])
    gps.append(data[20])

    print(f'CO Gas Concentration {co_gas_concentration}')

    print(f'HTS221_temperature {hts221_temperature}')
    print(f'HTS221_humidity {hts221_humidity}')
    print(f'LPS22HB_temperature {lps22hb_temperature}')
    print(f'LPS22HB_pressure {lps22hb_pressure}')
    print(f'Time_of_flight {time_of_flight}')
    print(
        f'Magnetometer {magnetometer[0]} {magnetometer[1]} {magnetometer[2]}')
    print(
        f'Accelerometer {acceleration[0]} {acceleration[1]} {acceleration[2]}')
    print(
        f'Gyroscope {gyroscope[0]} {gyroscope[1]} {gyroscope[2]}')
    print(
        f'Heart Rate {heartbeat[0]} Oxygen {heartbeat[1]}')
    print(
        f'Latitude {gps[0]} Longitude {gps[1]}')
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
