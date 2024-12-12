#ifndef Interpolation_h
#define Interpolation_h

#define UNDEFINED false
#define DEFINED true

struct Data {
    double expectedValue, recordedValue;
};

struct Index {
    int x1, x2,y1,y2;
};

struct LinearRegression {
    double sum_x1, sum_x2, sum_y1, sum_xy, constant, coeff;
};

struct Exponential {
    double sum_x1, sum_x2, sum_y1, sum_xy, coeff, base, A, B;
};

struct Power{
    double sum_x1, sum_x2, sum_y1, sum_xy, coeff, power, A;
};

class Interpolation
{
	private:
		Index indexStruct;
		LinearRegression linearRegressionStruct;
		Exponential exponentialStruct;
		Power powerStruct;

	protected:
        //Availability of
        bool linear_availability = false;
        bool linear_derivative_availability = false;

    public:
        Interpolation(Data data[], int array_size);
        virtual ~Interpolation();

        void interpolationUpdate(Data _data[]);
        void sizeUpdate(int _array_size);

        double linear(double _target);
        double getLinearDerivative(double _point);
        bool linearDerivativeAvailable();
        bool linearAvailable();

        double lagrange(double _target);

        double linearRegression(double _point);
        double getLinearRegressionDerivative();

        double exponentialFitting(double _point);
        double getExponentialBase();
        double getExponentialCoeff();

        double powerFitting(double _point);
        double getPowerCoeff();
        double getPowerPower();

		int _array_size_;

		//FIXME unbounded causes: assert failed: block_locate_free heap_tlsf.c:441 (block_size(block) >= size)
//        Data _data_[];
		Data _data_[5];
};

#endif
