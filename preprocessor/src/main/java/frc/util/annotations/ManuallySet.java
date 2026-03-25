package frc.util.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;

@Target({ElementType.FIELD, ElementType.LOCAL_VARIABLE})
public @interface ManuallySet {
     String value() default "__UNKNOWN__";
}
