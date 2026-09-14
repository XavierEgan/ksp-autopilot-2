import krpc

conn = krpc.connect()
vessel = conn.space_center.active_vessel
refframe = vessel.orbit.body.reference_frame
time_stream = conn.add_stream(getattr, conn.space_center, "ut")

i = 0
while True:
    print(f"{i}: {time_stream()}")
    i += 1