
class SlewRateLimiter {
public:
    SlewRateLimiter(double limit);
    double Calculate(double reading);
private:
    double limit_;
    double last_output_;
};