#ifndef FRACTION_H
#define FRACTION_H

struct fraction
{
  int denominator;
  int divisor;
  fraction() = default;
  fraction(int denom, int div);
};

fraction frac_add(fraction a, fraction b);
fraction frac_subtract(fraction a, fraction b);
fraction frac_multiply(fraction a, fraction b);
fraction frac_divide(fraction a, fraction b);
fraction frac_divide(fraction a, int b);
fraction frac_reduce(fraction a);
int gcd(int a, int b);
int gcd_func(int a, int b);
int fast_div(int a, int b);
int div_func(int a, int b);

fraction operator+(fraction a, fraction b);
fraction operator-(fraction a, fraction b);
fraction operator*(fraction a, fraction b);
fraction operator*(fraction a, double b);
fraction operator*(double b, fraction a);

fraction operator/(fraction a, fraction b);
fraction operator/(fraction a, int b);




#endif //FRACTION_H