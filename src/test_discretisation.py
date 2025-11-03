import discretisation as disc
import numpy as np

def test_euler_discretisation():
    A = np.array([[0, 1], [-2, -3]])
    B = np.array([[0], [1]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    ts = 0.1

    Ad_expected = np.array([[1, 0.1], [-0.2, 0.7]])
    Bd_expected = np.array([[0], [0.1]])
    Cd_expected = C
    Dd_expected = D

    Ad, Bd, Cd, Dd = disc.euler_discretise(A, B, C, D, ts)

    assert np.allclose(Ad, Ad_expected), "Ad does not match expected value"
    assert np.allclose(Bd, Bd_expected), "Bd does not match expected value"
    assert np.allclose(Cd, Cd_expected), "Cd does not match expected value"
    assert np.allclose(Dd, Dd_expected), "Dd does not match expected value"

def test_exact_discretisation():
    A = np.array([[0, 1], [-2, -3]])
    B = np.array([[0], [1]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    ts = 0.1

    Ad_expected = np.array([[0.99094409, 0.08610699],
                            [-0.17221398, 0.73262424]])
    Bd_expected = np.array([[0.00452796],
                            [0.08610699]])
    Cd_expected = C
    Dd_expected = D

    Ad, Bd, Cd, Dd = disc.exact_discretise(A, B, C, D, ts)

    assert np.allclose(Ad, Ad_expected), "Ad does not match expected value"
    assert np.allclose(Bd, Bd_expected), "Bd does not match expected value"
    assert np.allclose(Cd, Cd_expected), "Cd does not match expected value"
    assert np.allclose(Dd, Dd_expected), "Dd does not match expected value"