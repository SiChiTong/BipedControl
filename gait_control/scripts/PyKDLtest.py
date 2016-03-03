import unittest
import kinfamtest
import framestest
import frameveltest

suite = unittest.TestSuite()
suite.addTest(framestest.suite())
suite.addTest(frameveltest.suite())
suite.addTest(kinfamtest.suite())

unittest.TextTestRunner(verbosity=3).run(suite)