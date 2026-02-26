package frc.util.annotations;

import java.util.Set;
import java.util.regex.Pattern;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.Processor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.TypeElement;
import javax.tools.Diagnostic;

import com.google.auto.service.AutoService;

@AutoService(Processor.class)
@SupportedAnnotationTypes("frc.util.annotations.MagicNumber")
@SupportedSourceVersion(SourceVersion.RELEASE_17)
public class MagicNumberProcessor extends AbstractProcessor {

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        for(Element element : roundEnv.getElementsAnnotatedWith(MagicNumber.class)) {
            MagicNumber annotation = element.getAnnotation(MagicNumber.class);
            String origin = annotation.value();
            
            // Require a that the origin is set
            if(origin.equals("__UNKNOWN__")) {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.WARNING,
                    "Magic number found without an origin. Use @MagicNumber(origin = \"<Where the number came from>\")",
                    element
                );
            }
        }
        return true;
    }
}
