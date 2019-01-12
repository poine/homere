namespace homere_controller {
  LeftFeedForward::LeftFeedForward() {
    W1_ << -10.17006695, -7.13475475, -0.67335383;
    B1_ <<  0.44306333,-0.38489253, 3.62648456;
    W2_ <<  -6.72289203,  9.57973955,-11.40517566;
    B2_ <<  44.1416272;
  }
  RightFeedForward::RightFeedForward() {
    W1_ <<  -4.97471516e+02, -8.25912923e+02, -1.31224809e-02;
    B1_ <<  -51.90567223, 117.47677702,   1.80654168;
    W2_ <<   6.77110727e-02, -4.10062870e-02, -5.43095206e+02;
    B2_ <<  985.10687939;
  }
}