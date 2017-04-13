import time

class SetTime(object):
    OpCode = 0x02

    def __init__(self, time_to_set=None):
        if (time_to_set != None):
            self.epoch = time_to_set
        else:
            self.epoch = time.time()

    def serialize(self):
        ms = int(self.epoch % 1 * 1000)
        print("epoch = {epoch}".format(epoch=self.epoch))
        #print("Waiting {ms}ms".format(ms=(1000-ms)))
        return bytearray([self.OpCode]) + int(self.epoch).to_bytes(4, byteorder='little') + ms.to_bytes(2, byteorder='little')
