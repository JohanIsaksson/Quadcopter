#include "fraction.h"


fraction::fraction(int denom, int div)
{
  denominator = denom;
  divisor = div;
}

fraction frac_add(fraction a, fraction b)
{
  return fraction(a.denominator*b.divisor + b.denominator*a.divisor,
                  a.divisor * b.divisor);
}

fraction frac_subtract(fraction a, fraction b)
{
  return fraction(a.denominator*b.divisor - b.denominator*a.divisor,
                  a.divisor * b.divisor);
}

fraction frac_multiply(fraction a, fraction b)
{
  return fraction(a.denominator * b.denominator,
                  a.divisor * b.divisor);
}

fraction frac_multiply(fraction a, int b)
{
  return fraction(a.denominator * b, a.divisor);
}

fraction frac_divide(fraction a, fraction b)
{
  return fraction(a.denominator * b.divisor,
                  a.divisor * b.denominator);
}

fraction frac_divide(fraction a, int b)
{
  return fraction(fast_div(a.denominator, b),
                  fast_div(a.divisor, b));
}

fraction frac_reduce(fraction a)
{
  int div = gcd(a.denominator, a.divisor);
  return frac_divide(a, div);
}

int gcd(int a, int b)
{
  if (a >= 0 && b > 0) return gcd_func(a, b);
  if (a >= 0 && b < 0) return -gcd_func(a, -b);
  if (a < 0 && b > 0) return -gcd_func(-a, b);
  else return gcd_func(-a, -b);
}


int gcd_func(int a, int b)
{
  int m1 = a;
  int m2 = b;
  while(m1 != m2)
  {
    if (m1 > m2)
    {
      m1 -= m2;
    }
    else
    {
      m2 -= m1;
    }
  }
  return m1;
}

/*
 * Only works if a can be divided by b without leaving a remainder.
 * Returns INT_MAX or INT_MIN if divided by 0.
 */
int fast_div(int a, int b)
{
  if (a >= 0 && b == 0) return 2147483647;
  if (a < 0 && b == 0) return -2147483648;

  if (a >= 0 && b > 0) return div_func(a, b);
  if (a >= 0 && b < 0) return -div_func(a, -b);
  if (a < 0 && b > 0) return -div_func(-a, b);
  else return div_func(-a, -b);
}

int div_func(int a, int b)
{
  int sum = 0;
  int remainder = a;
  while (remainder >= b)
  {
    remainder -= b;
    sum++;
  }
  return sum;
}

// operators
fraction operator+(fraction a, fraction b)
{
  return frac_add(a, b);
}

fraction operator-(fraction a, fraction b)
{
  return frac_subtract(a, b);
}

fraction operator*(fraction a, fraction b)
{
  return frac_multiply(a, b);
}

fraction operator*(fraction a, double b)
{
  return frac_multiply(a, b);
}

fraction operator*(double b, fraction a)
{
  return frac_multiply(a, b);
}

fraction operator/(fraction a, fraction b)
{
  return frac_divide(a, b);
}

fraction operator/(fraction a, int b)
{
  return frac_divide(a, b);
}