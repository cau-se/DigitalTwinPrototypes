import time
import unittest

from simple_driver.driver import SimpleDriver, Observer


class TestSampleHandler(Observer):
    def __init__(self):
        Observer.__init__(self)
        self.samples = []

    def new_sample(self, sample: dict) -> None:
        self.samples.append(sample)


class TestDriverIntegration(unittest.TestCase):

    def setUp(self) -> None:
        self.driver = SimpleDriver()
        self.driver.start('/dev/ttyUSB0', 0)
        self.sample_observer = TestSampleHandler()
        time.sleep(0.5)

    def test_get_sample(self):
        self.sample_observer.observe('get_sample', self.sample_observer.new_sample)
        self.driver.get_sample()
        time.sleep(1)
        print(self.sample_observer.samples)
        self.assertTrue(len(self.sample_observer.samples) > 0)

    def tearDown(self) -> None:
        self.driver.stop_driver()


class TestDriverIntegrationInterval(unittest.TestCase):

    def setUp(self) -> None:
        self.driver = SimpleDriver()
        self.driver.start('/dev/ttyUSB0', 0)
        self.sample_observer = TestSampleHandler()
        self.sample_observer.observe('get_sample', self.sample_observer.new_sample)
        time.sleep(0.5)

    def test_period_start(self):
        self.driver.set_interval(0.5)
        time.sleep(3)
        print(self.sample_observer.samples)
        time.sleep(1)
        self.driver.set_interval(0)
        self.assertTrue(len(self.sample_observer.samples) > 3)

    def test_period_stop(self):
        self.driver.set_interval(0.5)
        time.sleep(2)
        self.driver.set_interval(0)
        size_now = len(self.sample_observer.samples)
        self.assertEqual(len(self.sample_observer.samples), size_now)

    def tearDown(self) -> None:
        self.driver.stop_driver()


def suite():
    suite = unittest.TestSuite()
    suite.addTest(TestDriverIntegration('test_get_sample'))
    suite.addTest(TestDriverIntegrationInterval('test_set_interval'))
    return suite


if __name__ == '__main__':
    runner = unittest.TextTestRunner()
    runner.run(suite())
