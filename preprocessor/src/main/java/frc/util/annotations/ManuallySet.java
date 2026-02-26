package frc.util.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Target;

@Target({ElementType.FIELD, ElementType.LOCAL_VARIABLE})
public @interface ManuallySet {
     String date() default "__UNKNOWN__";
}
