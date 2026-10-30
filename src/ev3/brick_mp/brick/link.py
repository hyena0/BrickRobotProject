# brick-sw-version: 5B.14
"""Bluetooth mailbox link between the bricks (left brick is server).

Boxes carry JSON text: cmd goes master to slave, reply, state and evt
go slave to master, hb (heartbeat plus stop flag) goes master to slave.
FakeLink gives the same box shape to PC side tests.
"""

CMD_BOX = "brick-cmd"
REPLY_BOX = "brick-reply"
STATE_BOX = "brick-state"
EVT_BOX = "brick-evt"
HB_BOX = "brick-hb"


class LinkEnd:
    def __init__(self):
        self.cmd = None
        self.reply = None
        self.state = None
        self.evt = None
        self.hb = None


class FakeMailbox:
    """Latest value box with the TextMailbox send and read shape."""

    def __init__(self):
        self._value = None

    def send(self, value):
        self._value = value

    def read(self):
        return self._value


class FakeLink(LinkEnd):
    """In-memory link shared by both runtimes in PC side tests."""

    def __init__(self):
        LinkEnd.__init__(self)
        self.cmd = FakeMailbox()
        self.reply = FakeMailbox()
        self.state = FakeMailbox()
        self.evt = FakeMailbox()
        self.hb = FakeMailbox()


def open_master_link():
    from pybricks.messaging import BluetoothMailboxServer, TextMailbox
    server = BluetoothMailboxServer()
    end = LinkEnd()
    end.cmd = TextMailbox(CMD_BOX, server)
    end.reply = TextMailbox(REPLY_BOX, server)
    end.state = TextMailbox(STATE_BOX, server)
    end.evt = TextMailbox(EVT_BOX, server)
    end.hb = TextMailbox(HB_BOX, server)
    end._conn = server
    server.wait_for_connection()
    return end


def open_slave_link(server_addr):
    from pybricks.messaging import BluetoothMailboxClient, TextMailbox
    client = BluetoothMailboxClient()
    end = LinkEnd()
    end.cmd = TextMailbox(CMD_BOX, client)
    end.reply = TextMailbox(REPLY_BOX, client)
    end.state = TextMailbox(STATE_BOX, client)
    end.evt = TextMailbox(EVT_BOX, client)
    end.hb = TextMailbox(HB_BOX, client)
    end._conn = client
    end._addr = server_addr
    client.connect(server_addr)
    return end


def reconnect_slave_link(end):
    end._conn.connect(end._addr)
